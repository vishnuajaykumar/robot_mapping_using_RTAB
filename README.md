This project involves using RTAB-Map, a real-time appearance-based SLAM algorithm, to build a 3D map of an environment. RTAB-Map relies on a bag-of-words approach to recognize previously visited locations (loop closures), which helps correct drift and improve the map’s accuracy. By detecting these loop closures and updating the global map accordingly, the system can create a more robust and consistent representation of the environment in real time.

![image](https://github.com/user-attachments/assets/872f1354-e0d3-48f9-9fbe-11e75fadffe0)



Download the database file from : https://drive.google.com/file/d/1ytW7F4Jsjz-zg7lbeNWSw2m085uv2uIM/view?usp=drive_link 

and paste it in your <your_workspace>/src/ball_chaser/maps for localization to function correctly 
