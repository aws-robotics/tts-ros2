# tts


## Overview
The `tts` ROS2 node enables a robot to speak with a human voice by providing a Text-To-Speech service.
Out of the box this package listens to a speech topic, submits text to the Amazon Polly cloud service to generate an audio stream file,
retrieves the audio stream from Amazon Polly, and plays the audio via the default output device.
The nodes can be configured to use different voices as well as custom lexicons and SSML tags which enable you to control aspects of speech,
such as pronunciation, volume, pitch, speed rate, etc. A [sample ROS application] with this node,
and more details on speech customization are available within the [Amazon Polly documentation].

**Amazon Polly Summary**: Amazon Polly is a service that turns text into lifelike speech, allowing you to create applications that talk,
and build entirely new categories of speech-enabled products. Amazon Polly is a Text-to-Speech service that uses advanced deep learning technologies to synthesize speech that sounds like a human voice.
With dozens of lifelike voices across a variety of languages, you can select the ideal voice and build speech-enabled applications that work in many different countries.

**Features in Active Development**:
- Offline TTS

### License
The source code is released under an [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com

### Supported ROS Distributions
- Crystal


## Installation

### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files] helpful.

This node will require the following AWS account IAM role permissions:
- `polly:SynthesizeSpeech`

### Build and Test

#### Build from Source

Create a ROS workspace and a source directory

    mkdir -p ~/ros-workspace/src

To install from source, clone the latest version from master branch and compile the package

- Clone the package into the source directory

        cd ~/ros-workspace/src 
        git clone https://github.com/aws-robotics/tts-ros2.git

- Install dependencies

        cd ~/ros-workspace && sudo apt-get update 
        rosdep install --from-paths src --ignore-src -r -y

- Install the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library Path

        source ~/ros-workspace/install/local_setup.bash
        
- Build and run the unit tests

        colcon test --packages-select tts && colcon test-result --all

#### Test on Containers/Virtual Machines

Even if your container or virtual machine does not have audio device, you can still test TTS by leveraging an audio server.

The following is an example setup on a MacBook with PulseAudio as the audio server.
If you are new to PulseAudio, you may want to read the [PulseAudio Documentation].

**Step 1: Start PulseAudio on your laptop**

After installation, start the audio server with *module-native-protocol-tcp* loaded:

    pulseaudio --load=module-native-protocol-tcp --exit-idle-time=-1 --log-target=stderr -v

Note the extra arguments `-v` and `--log-target` are used for easier troubleshooting.

**Step 2: Run TTS nodes in container**

In your container, make sure you set the right environment variables.
For example, you can start the container using `docker run -it -e PULSE_SERVER=docker.for.mac.localhost ubuntu:16.04`.

Then you will be able to run ROS nodes in the container and hear the audio from your laptop speakers.

**Troubleshooting**

If your laptop has multiple audio output devices, make sure the right one has the right volume.
This command will give you a list of output devices and tell you which one has been selected:

    pacmd list-sinks | grep -E '(index:|name:|product.name)'

## Launch Files
An example launch file called `tts.launch.py` is provided.


## Usage

### Run the node
- **Plain text**
  - `ros2 launch tts tts.launch.py`
  - `ros2 run tts voicer 'Hello World'`

- **SSML**
  - `ros2 launch tts tts.launch.py`
  - `ros2 run tts voicer '<speak>Mary has a <amazon:effect name="whispered">little lamb.</amazon:effect></speak>' '{"text_type":"ssml"}'`

## Nodes

### polly_node
Polly node is the engine for the synthesizing job. It provides user-friendly yet powerful APIs so a user doesn't have to deal with technical details of AWS service calls.

#### Services
- **`polly`**

  Call the service to use Amazon Polly to synthesize the audio.

### synthesizer_node

#### Services
- **`synthesizer`**

  Call the service to synthesize.

#### Parameters

- **`text (string)`**

    The text to be synthesized.

- **`metadata (string, JSON format)`**

    Optional, for user to have control over how synthesis happens.

## Bugs & Feature Requests
Please contact the team directly if you would like to request a feature.

Please report bugs in [Issue Tracker].


[AWS Configuration and Credential Files]: https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html
[Amazon Polly documentation]: https://docs.aws.amazon.com/polly/latest/dg/what-is.html
[Amazon Web Services (AWS)]: https://aws.amazon.com/
[Apache 2.0]: https://aws.amazon.com/apache-2-0/
[Issue Tracker]: https://github.com/aws-robotics/tts-ros2/issues
[PulseAudio Documentation]: https://www.freedesktop.org/wiki/Software/PulseAudio/Documentation/
[official Amazon Polly document]: https://docs.aws.amazon.com/polly/latest/dg/voicelist.html
[sample ROS application]: https://github.com/aws-robotics/aws-robomaker-sample-application-voiceinteraction
