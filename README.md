# MKR1000

This repository stores completed code for a prototype IoT demo. More information available on my LinkedIn.

Internet of Things (IoT) is the process of connecting the physical world to the Internet. From thermostats to teddy bears to speakers to rotis, IoT devices have been at the forefront of the digital age, impacting every facet of life. By 2020, there will be 20.4 billion smart devices connected to the IoT; in addition, there is about an $11 trillion potential yearly impact for IoT technology by 2025 [1]. By leveraging IoT in industry, governments can monitor water consumption for residential and commercial use, oil and gas companies can optimize on-field production through sensors on their wells, and truck manufacturers can fit trucks with sensors to predict maintenance and order parts if necessary [2], just to give a few examples. 

Thanks to open-source platforms and the availability of cheaply manufactured electronic parts, building IoT devices is affordable for personal projects and for small-scale use cases. In college, I was exposed to Arduino - an open-source electronic prototyping platform. When I started building IoT devices, I used an Arduino Uno and other add-ons to enable internet connectivity to the microcontroller. There are several third party options available to add such connectivity (called Wi-Fi shields), such as the Onion Omega 2.0, PHPoC, and Google's selection of IoT boards. That being said, Arduino has come out with an incredible IoT microcontroller called the MKR1000, which I have since been using for work and for personal projects. This board has an embedded Wi-Fi shield that gives it the capability of a normal Arduino (specifically the Arduino Zero), but adds Internet connectivity. 

In this post, I want to empower people to build their own IoT devices. Specifically, I was tasked with providing an IoT solution to attach to a shelf robot that could assist humans in an factory or warehouse with their everyday tasks. My vision for the project is influenced from my time working as a quality control intern in a latex plant. I noticed that workers would have to manually transport containers, weigh them, and record results accurately - all while working in noisy or intense heat. The inspiration for the use of each sensor correlates to use cases that would keep human workers safe and would save plants money on human error. For example, the addition of distance and pressure sensors allow for the robot to detect if a container of sample material has spilled on its way to a QC technician and allow for weight to be automatically recorded and sent to a database for them to use without needing to wait for the container itself. In addition, the use of a humidity and temperature sensor along with a sound sensor augment the robot to look out for its human coworkers, alert the plant managers of any anomalies, and reduce the risk of human error. I was able to build an attachment for a robot at work that could detect the temperature of a container, the presence of the container, the ambient temperature, and the environmental noise levels. If there were an anomaly, the set-up would inform the user through an HTTP POST request to our company’s server. The MKR1000 gathers sensor data every ten seconds and alerts the user of any anomalies. In addition, the MKR1000 makes HTTP POST requests to a pre-defined endpoint (server) and can successfully handle responses. 
