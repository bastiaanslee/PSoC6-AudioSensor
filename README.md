# PSoC6-AudioSensor

Programmed with Eclipse IDE for ModusToolbox on a Cypress PSoC6 board.
This code does record audio samples with the PDM microphone, converts that to PCM data. Which is then going through a Fast Fourier Transform with HANN windowing, to be able to split the audio data into octaves. Based on A/C/Z weighting, the audio data is then further calculated into data that represents what our ears do actually hear.

Read all documentation, find details about the code and discuss optimisations for this project: https://www.electromaker.io/project/view/PSoC6-AudioSensor
And watch the YouTube video that goes through this as well, including a demo: https://youtu.be/JtMAm2YC7Pk


Credits go to all these pages that did help me getting to understand parts of it, summing together to this project. I've read so many pages, that I'm I did forgot some of them to list here.

Information about Sound, Audio and processing it:
* Wikipedia: Your ears https://en.wikipedia.org/wiki/Ear
* Wikipedia: Fourier Transform https://en.wikipedia.org/wiki/Fourier_transform
* Wikipedia: Joseph Fourier https://en.wikipedia.org/wiki/Joseph_Fourier
* Wikipedia: Window functions https://en.wikipedia.org/wiki/Window_function
* Wikipedia: Hann Window https://en.wikipedia.org/wiki/Hann_function
* NIH: Journey of Sound to the Brain https://upload.wikimedia.org/wikipedia/commons/7/72/Journey_of_Sound_to_the_Brain.ogv
* NTI-audio: Frequency-Weightings for Sound Level Measurements https://www.nti-audio.com/en/support/know-how/frequency-weightings-for-sound-level-measurements
* 3Blue1Brown: What is the Fourier Transform? A visual introduction https://youtu.be/spUNpyF58BY
* Steve L. Brunton: Fourier Analysis series https://www.youtube.com/playlist?list=PLMrJAkhIeNNT_Xh3Oy0Y4LTj0Oxo8GqsC
* Elan Ness-Cohn: Developing An Intuition for Fourier Transforms https://sites.northwestern.edu/elannesscohn/2019/07/30/developing-an-intuition-for-fourier-transforms/
* Stack Overflow https://stackoverflow.com/a/4678313 and https://stackoverflow.com/a/604756

Technical information and code examples:
* FreeRTOS: https://www.freertos.org/
* FreeRTOS Tasks: https://www.freertos.org/taskandcr.html
* FreeRTOS Queues: https://www.freertos.org/Embedded-RTOS-Queues.html
* Cypress documentation: WiFi Connection Manager Library: https://cypresssemiconductorco.github.io/wifi-connection-manager/api_reference_manual/html/index.html
* Cypress documentation: Secure Sockets: https://cypresssemiconductorco.github.io/secure-sockets/api_reference_manual/html/index.html
* Cypress documentation: Hardware Abstraction Layer (for PDM/PCM, RTC): https://cypresssemiconductorco.github.io/psoc6hal/html/index.html
* Ed Boel: Noice Level Meter https://bitbucket.org/edboel/edboel/src/master/noise/src/
* TTNApeldoorn: LoRaSoundkit https://github.com/TTNApeldoorn/sound-sensor
* Enrique Condes: arduinoFFT https://github.com/kosme/arduinoFFT
* David Lettier: NTP client: https://lettier.github.io/posts/2016-04-26-lets-make-a-ntp-client-in-c.html
* Nicolas Seriot: NTP datagram: http://seriot.ch/ntp.php#21
* Wikipedia: ASCI escape codes: https://en.wikipedia.org/wiki/ANSI_escape_code
* InfluxDB: Write data with the InfluxDB API: https://docs.influxdata.com/influxdb/v2.0/write-data/developer-tools/api/
