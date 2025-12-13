# search-support-robots

sudo apt install python3
sudo apt install python3.12-venv
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt


### To use mic array
Make sure requirements are installed
https://alphacephei.com/vosk/models
https://zadig.akeo.ie/
git clone https://github.com/respeaker/usb_4_mic_array.git
Change response.tostring() to response.tobytes() in tuning
python mic_array.py --model "/home/callum/ros2_dev_ws/src/search-support-robots-mic_array/models/vosk-model-small-en-us-0.15" --rate 16000 --show-partials --show-doa
