import sys, time, argparse, queue, json, threading
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch

TARGET_SR = 16000
DEVICE_MATCH_KEYS = ["respeaker", "usb mic array", "seeed", "xmos"]
PREFERRED_CHANNELS = [6, 8, 4, 2, 1]
BEAMFORMED_DEFAULT_INDEX = 5
KEYWORDS = ["stop", "help"]

def pick_respeaker_device():
    devices = sd.query_devices()
    hostapis = sd.query_hostapis()
    def hostapi_name(idx):
        try: return hostapis[devices[idx]['hostapi']]['name']
        except Exception: return "?"
    candidates = []
    for i, d in enumerate(devices):
        name = (d.get('name') or '').lower()
        if d.get('max_input_channels', 0) > 0 and any(k in name for k in DEVICE_MATCH_KEYS):
            candidates.append((i, d))
    if not candidates:
        di = sd.default.device[0]
        dn = devices[di]['name']
        print(f"Couldn't match a ReSpeaker by name. Using default input: '{dn}' ({hostapi_name(di)})")
        return di
    candidates.sort(key=lambda t: t[1].get('max_input_channels', 0), reverse=True)
    i, d = candidates[0]
    print(f"Using input: '{d['name']}' via {hostapi_name(i)} ({d.get('max_input_channels', 0)} ch max)")
    return i

def choose_channel_count(device_index):
    max_ch = sd.query_devices(device_index)['max_input_channels']
    for ch in PREFERRED_CHANNELS:
        if ch <= max_ch: return ch
    return max(1, max_ch)

def make_resampler(in_sr, out_sr):
    if in_sr == out_sr: return lambda x: x
    ratio = out_sr / in_sr
    def resample(x):
        x = np.asarray(x)
        if x.ndim == 1:
            n_in = x.shape[0]; n_out = int(np.round(n_in * ratio))
            t_old = np.linspace(0.0, 1.0, num=n_in, endpoint=False)
            t_new = np.linspace(0.0, 1.0, num=n_out, endpoint=False)
            return np.interp(t_new, t_old, x).astype(np.float32)
        n_in = x.shape[0]; n_out = int(np.round(n_in * ratio))
        t_old = np.linspace(0.0, 1.0, num=n_in, endpoint=False)
        t_new = np.linspace(0.0, 1.0, num=n_out, endpoint=False)
        out = np.empty((n_out, x.shape[1]), dtype=np.float32)
        for c in range(x.shape[1]):
            out[:, c] = np.interp(t_new, t_old, x[:, c])
        return out
    return resample

def main():
    ap = argparse.ArgumentParser(description="ReSpeaker live STT + DOA (Vosk)")
    ap.add_argument("--model", required=True, help="Path to an unzipped Vosk model dir")
    ap.add_argument("--rate", type=int, default=TARGET_SR, help="Capture sample rate (resampled to 16 kHz for Vosk)")
    ap.add_argument("--device", default=None, help="Device index or name snippet")
    ap.add_argument("--channel", type=int, default=None, help="0-based channel for STT input")
    ap.add_argument("--no-led", action="store_true", help="Disable LED ring even if available")
    ap.add_argument("--show-partials", action="store_true", help="Print partial words while speaking")
    ap.add_argument("--show-doa", action="store_true", help="Poll and print Direction of Arrival (0–359°)")
    ap.add_argument("--doa-interval", type=float, default=0.4, help="Seconds between DOA polls")
    ap.add_argument("--plot-doa", action="store_true", help="Show live DOA arrow plot")
    args = ap.parse_args()

    pixel = None
    if not args.no_led:
        try:
            from pixel_ring import usb_pixel_ring_v2, usb_pixel_ring
            try: pixel = usb_pixel_ring_v2.UsbPixelRingV2()
            except Exception: pixel = usb_pixel_ring.UsbPixelRing()
            try:
                pixel.set_brightness(16)
                pixel.wakeup()
            except Exception:
                pixel = None
        except Exception:
            pixel = None

    if args.device is None:
        device_index = pick_respeaker_device()
    else:
        try:
            device_index = int(args.device)
        except ValueError:
            device_index = None
            for i, d in enumerate(sd.query_devices()):
                if args.device.lower() in (d.get('name') or '').lower():
                    device_index = i; break
            if device_index is None:
                print(f"Couldn't find device containing '{args.device}'."); sys.exit(1)

    channels = choose_channel_count(device_index)
    in_sr = args.rate
    ch_idx = (int(args.channel) if args.channel is not None
              else (BEAMFORMED_DEFAULT_INDEX if channels >= 6 else 0))
    if ch_idx < 0 or ch_idx >= channels:
        print(f"Channel index {ch_idx} out of range for {channels} channels."); sys.exit(1)

    calib_secs = 1.5
    tmp = []
    def calib_cb(indata, frames, time_info, status):
        if status: print(f"[calib status] {status}")
        tmp.append(indata.copy())
    print("Calibrating levels for", calib_secs, "s...")
    with sd.InputStream(device=device_index, channels=channels, samplerate=in_sr, dtype="float32",
                        blocksize=int(in_sr // 10), callback=calib_cb):
        sd.sleep(int(calib_secs * 1000))
    calib = np.concatenate(tmp, axis=0) if tmp else np.zeros((int(in_sr * calib_secs), channels), np.float32)
    rms = np.sqrt((calib ** 2).mean(axis=0) + 1e-12)
    best = int(np.argmax(rms))
    print("sPer-channel RMS:", ["{:.3f}".format(x) for x in rms])
    if args.channel is None:
        ch_idx = best
        print(f"Auto-selecting channel #{ch_idx}")

    try:
        from vosk import Model, KaldiRecognizer
    except Exception:
        print("Vosk not installed. Run:  pip install vosk"); sys.exit(1)
    try:
        model = Model(args.model)
    except Exception as e:
        print(f"Failed to load Vosk model at '{args.model}': {e}"); sys.exit(1)
    recognizer = KaldiRecognizer(model, TARGET_SR)
    recognizer.SetWords(True)

    current_doa = {"angle": None}
    doa_lock = threading.Lock()
    stop_doa = threading.Event()
    doa_thread = None

    def get_doa_str():
        with doa_lock:
            ang = current_doa["angle"]
        return f"{ang:03d}°" if isinstance(ang, int) else "NA°"

    if args.show_doa:
        tuner = None
        try:
            import usb.core
            dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            if dev is None:
                raise RuntimeError("ReSpeaker HID device not found via PyUSB")
            try:
                from tuning import Tuning
            except Exception:
                from tuning import Tuning
            tuner = Tuning(dev)
        except Exception as e:
            print(" DOA unavailable:", e)
            print(" Ensure: WinUSB via Zadig on the HID interface, pyusb installed, and libusb-1.0.dll on PATH.")

        if tuner is not None:
            def poll_doa():
                last = None
                while not stop_doa.is_set():
                    try:
                        angle = int(tuner.direction)
                        with doa_lock:
                            current_doa["angle"] = angle
                        if pixel is not None and angle != last:
                            try: pixel.set_direction(angle)
                            except Exception: pass
                        last = angle
                    except Exception:
                        pass
                    time.sleep(max(0.05, args.doa_interval))
            doa_thread = threading.Thread(target=poll_doa, daemon=True)
            doa_thread.start()
            print("DOA polling: enabled")

    # Setup live plot if requested
    fig, ax, arrow_patch, keyword_arrow, text_display = None, None, None, None, None
    if args.plot_doa and args.show_doa:
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 10), subplot_kw={'projection': 'polar'})
        ax.set_theta_zero_location('N')  # 0° at top (north)
        ax.set_theta_direction(-1)  # Clockwise
        ax.set_ylim(0, 1)
        
        # Add confidence rings at 25%, 50%, 75%, 100%
        ax.set_yticks([0.25, 0.5, 0.75, 1.0])
        ax.set_yticklabels(['25%', '50%', '75%', '100%'], fontsize=10)
        ax.grid(True, alpha=0.3, linestyle='--')
        
        # Draw more prominent rings
        theta_circle = np.linspace(0, 2*np.pi, 100)
        for radius, alpha in [(0.25, 0.3), (0.5, 0.4), (0.75, 0.5), (1.0, 0.6)]:
            ax.plot(theta_circle, [radius]*len(theta_circle), 'k-', alpha=alpha, linewidth=1)
        
        # Add simple robot model at center of polar plot
        # Robot body - gray rectangle centered at origin
        robot_width = 0.08   # in radial units
        robot_height = 0.12
        
        # Create robot body as a polygon in polar coordinates
        # Define corners of rectangle centered at origin
        rect_corners_x = np.array([-robot_width/2, robot_width/2, robot_width/2, -robot_width/2, -robot_width/2])
        rect_corners_y = np.array([-robot_height/2, -robot_height/2, robot_height/2, robot_height/2, -robot_height/2])
        
        # Convert to polar coordinates (theta, r)
        rect_r = np.sqrt(rect_corners_x**2 + rect_corners_y**2)
        rect_theta = np.arctan2(rect_corners_x, rect_corners_y)
        
        robot_body = ax.fill(rect_theta, rect_r, 
                            facecolor='darkgray', edgecolor='black', 
                            linewidth=2, zorder=20)
        
        # Wheels - 4 black rectangles
        wheel_width = 0.02
        wheel_height = 0.04
        
        wheel_positions = [
            (-robot_width/2 - wheel_width/2, robot_height/2 - wheel_height/2),   # Top-left
            (robot_width/2 + wheel_width/2, robot_height/2 - wheel_height/2),    # Top-right
            (-robot_width/2 - wheel_width/2, -robot_height/2 + wheel_height/2),  # Bottom-left
            (robot_width/2 + wheel_width/2, -robot_height/2 + wheel_height/2)    # Bottom-right
        ]
        
        for wx, wy in wheel_positions:
            # Create wheel rectangle
            wheel_x = np.array([wx - wheel_width/2, wx + wheel_width/2, 
                               wx + wheel_width/2, wx - wheel_width/2, wx - wheel_width/2])
            wheel_y = np.array([wy - wheel_height/2, wy - wheel_height/2, 
                               wy + wheel_height/2, wy + wheel_height/2, wy - wheel_height/2])
            
            wheel_r = np.sqrt(wheel_x**2 + wheel_y**2)
            wheel_theta = np.arctan2(wheel_x, wheel_y)
            
            ax.fill(wheel_theta, wheel_r, 
                   facecolor='black', edgecolor='black',
                   linewidth=1, zorder=21)
        
        ax.set_title("Direction of Arrival (Confidence)", pad=20, fontsize=14, fontweight='bold')
        
        # Create regular speech arrow (GREEN)
        arrow_patch = FancyArrowPatch((0, 0), (0, 0.5),
                             arrowstyle='->,head_width=0.4,head_length=0.3',
                             color='green', linewidth=3, 
                             transform=ax.transData,
                             mutation_scale=20,
                             visible=False)
        ax.add_patch(arrow_patch)

        arrow_label = ax.text(0, 0.55, '', ha='center', va='bottom', 
                        fontsize=10, fontweight='bold', 
                        color='green', visible=False)
        
        # Create keyword arrow (RED)
        keyword_arrow = FancyArrowPatch((0, 0), (0, 0.5),
                             arrowstyle='->,head_width=0.4,head_length=0.3',
                             color='red', linewidth=3, 
                             transform=ax.transData,
                             mutation_scale=20,
                             visible=False,
                             zorder=10)  # Draw on top
        ax.add_patch(keyword_arrow)

        keyword_label = ax.text(0, 0.55, '', ha='center', va='bottom',
                        fontsize=10, fontweight='bold',
                        color='red', visible=False, zorder=11)
        
        # Create text display at bottom of figure
        # Use figure coordinates (0.5 = center horizontally, lower value = lower position)
        text_display = fig.text(0.5, 0.01, 'Listening...', 
                               ha='center', va='bottom',
                               fontsize=12, fontweight='bold',
                               wrap=True,
                               bbox=dict(boxstyle='round,pad=0.5', 
                                       facecolor='lightgray', 
                                       edgecolor='black',
                                       alpha=0.8))
                
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.1)

    q = queue.Queue()
    resample = make_resampler(in_sr, TARGET_SR)
    
    def wrap_text(text, max_chars_per_line=80):
        """Wrap text to multiple lines if it exceeds max_chars_per_line"""
        words = text.split()
        lines = []
        current_line = []
        current_length = 0
        
        for word in words:
            word_length = len(word)
            # +1 for the space
            if current_length + word_length + len(current_line) > max_chars_per_line and current_line:
                lines.append(' '.join(current_line))
                current_line = [word]
                current_length = word_length
            else:
                current_line.append(word)
                current_length += word_length
        
        if current_line:
            lines.append(' '.join(current_line))
        
        return '\n'.join(lines)

    def callback(indata, frames, time_info, status):
        if status: print(f"\n[stream status] {status}", file=sys.stderr)
        x = indata.astype(np.float32, copy=False)
        mono = x[:, ch_idx] if x.ndim == 2 else x
        mono_rs = resample(mono)
        pcm16 = (np.clip(mono_rs, -1.0, 1.0) * 32767.0).astype(np.int16).tobytes()
        q.put(pcm16)

    print(f"Opening input stream on device={device_index}, channels={channels}, rate={in_sr} Hz")
    print(f"Using Vosk model at: {args.model}")
    print(f"Feeding channel #{ch_idx}")
    print("Speak!  (Ctrl+C to stop)\n")

    try:
        with sd.InputStream(device=device_index, channels=channels, samplerate=in_sr,
                            dtype="float32", blocksize=int(in_sr // 10), callback=callback):
            if pixel is not None:
                try: pixel.wakeup()
                except Exception: pass
            buffer_concat = b""
            bite_bytes = int(TARGET_SR * 0.25) * 2
            while True:
                chunk = q.get()
                buffer_concat += chunk
                if len(buffer_concat) >= bite_bytes:
                    if recognizer.AcceptWaveform(buffer_concat):
                        res = json.loads(recognizer.Result())
                        words = res.get("result", [])
                        if words:
                            # Check if any keyword was detected
                            detected_keywords = []
                            # Build the full phrase from all words
                            phrase = ' '.join([w.get("word", "") for w in words])
                            
                            for w in words:
                                wd = w.get("word", "").lower()
                                cf = w.get("conf")
                                print(f"[{get_doa_str()}] {wd}  (conf={cf:.2f})")
                                
                                if wd in KEYWORDS:
                                    detected_keywords.append((wd, cf))
                            
                            # Update plot if enabled
                            if args.plot_doa and fig is not None:
                                with doa_lock:
                                    ang = current_doa["angle"]
                                
                                if isinstance(ang, int):
                                    theta = np.deg2rad(ang)
                                    
                                    # Only update arrows if keywords detected
                                    if detected_keywords:
                                        # Use highest confidence keyword
                                        max_conf = max(cf for _, cf in detected_keywords)
                                        keyword_length = max(0.1, min(1.0, max_conf)) + 0.01
                                        
                                        # Print alert
                                        for kw, cf in detected_keywords:
                                            print(f"KEYWORD DETECTED: '{kw.upper()}' at {ang}° (conf={cf:.2f})")
                                        
                                        keyword_name = detected_keywords[0][0].upper()

                                        keyword_arrow.remove()
                                        keyword_arrow = FancyArrowPatch((theta, 0), (theta, keyword_length),
                                                                       arrowstyle='->,head_width=0.4,head_length=0.3',
                                                                       color='red', linewidth=3, 
                                                                       transform=ax.transData,
                                                                       mutation_scale=20,
                                                                       visible=True,
                                                                       zorder=10)
                                        ax.add_patch(keyword_arrow)
                                        
                                        keyword_label.set_position((theta, keyword_length + 0.08))
                                        keyword_label.set_text(f'{keyword_name}\n{ang}°')
                                        keyword_label.set_visible(True)
                                        
                                        # Update text display with keyword phrase
                                        # Color based on confidence
                                        if text_display is not None:
                                            if max_conf < 0.5:
                                                box_color = 'mistyrose'  # Low confidence - light red
                                            elif max_conf < 0.75:
                                                box_color = 'lightyellow'  # Medium confidence - orange
                                            else:
                                                box_color = 'lightgreen'  # High confidence - red
                                            
                                            display_text = wrap_text(f'{phrase.upper()} (keyword conf: {max_conf:.2f})')
                                            text_display.set_text(display_text)
                                            text_display.get_bbox_patch().set_facecolor(box_color)

                                        fig.canvas.draw()
                                        fig.canvas.flush_events()
                                    else:
                                        # No keywords - update regular arrow only
                                        # Calculate average confidence
                                        if words:
                                            avg_conf = sum(w.get("conf", 0.0) for w in words) / len(words)
                                        else:
                                            avg_conf = 0.0
                                        
                                        # Update regular arrow (always GREEN)
                                        arrow_length = max(0.1, min(1.0, avg_conf)) + 0.01
                                        
                                        arrow_patch.remove()
                                        arrow_patch = FancyArrowPatch((theta, 0), (theta, arrow_length),
                                                                     arrowstyle='->,head_width=0.4,head_length=0.3',
                                                                     color='green', linewidth=3, 
                                                                     transform=ax.transData,
                                                                     mutation_scale=20,
                                                                     visible=True)
                                        ax.add_patch(arrow_patch)

                                        arrow_label.set_position((theta, arrow_length + 0.08))
                                        arrow_label.set_text(f'Speech\n{ang}°')
                                        arrow_label.set_color('green')
                                        arrow_label.set_visible(True)
                                        
                                        # Update text display with phrase
                                        # Color based on confidence
                                        if text_display is not None:
                                            if avg_conf < 0.5:
                                                box_color = 'mistyrose'  # Low confidence - light pink
                                            elif avg_conf < 0.75:
                                                box_color = 'lightyellow'  # Medium confidence - light yellow
                                            else:
                                                box_color = 'lightgreen'  # High confidence - light green
                                            
                                            display_text = wrap_text(f'{phrase} (avg. conf: {avg_conf:.2f})')
                                            text_display.set_text(display_text)
                                            text_display.get_bbox_patch().set_facecolor(box_color)
                                        
                                        fig.canvas.draw()
                                        fig.canvas.flush_events()
                    else:
                        # partials
                        pres = json.loads(recognizer.PartialResult())
                        part = pres.get("partial", "").strip()
                        if part and args.show_partials:
                            print(f"[{get_doa_str()}] {part}", end="\r", flush=True)
                    buffer_concat = b""
 
    except KeyboardInterrupt:
        pass
    finally:
        if pixel is not None:
            try: pixel.think(); time.sleep(0.5); pixel.off()
            except Exception: pass
        if doa_thread is not None:
            stop_doa.set(); doa_thread.join(timeout=1.0)
        if fig is not None:
            plt.close(fig)

    try:
        final = json.loads(recognizer.FinalResult())
        text = final.get("text", "").strip()
        if text:
            print(f"\n[{get_doa_str()}] {text}")
    except Exception:
        pass
    print("\nStopped.")

if __name__ == "__main__":
    main()