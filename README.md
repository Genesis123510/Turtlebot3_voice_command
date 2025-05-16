Create a python environment:
$ python3 -m venv ~/coqui-venv

Activate the environment:
$ source ~/coqui-venv/bin/activate

Install coqui-tts:
$ pip install coqui-tts

Clone this repo on your workspace e.g ros2 inside the src file:
$ git clone https://github.com/Genesis123510/coqui_speaker

Run the TTS using:
    python3 ~/<workspace_name>/src/coqui_speaker/coqui_speaker/coqui_speaker_node.py

Run the STT using:
    ros2 launch voskros voskros.launch.yaml model:=en-us-0.15

Run voice command node:
    ros2 run turtlebot3_voice_command voice_command_node

Note: since the voice command package couldnt not be pushed all at once, please extract all files in your own workspace
