// File name:  dronepanel.cpp
// Description: implementation related to custom rviz panel for drone viewpoints
// Author: Mike Hagenow
// Date: 2/11/22
// Using information found in the ROS
// custom plugins tutorial: https://github.com/ros-visualization/visualization_tutorials/blob/groovy-devel/rviz_plugin_tutorials/src/teleop_panel.cpp

#include <stdio.h>
#include "dronepanel.h"

#include <QWidget>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFont>
#include <QSizePolicy>
#include <QString>


#include <ros/console.h>

#include <rviz/visualization_manager.h>
#include <rviz/view_controller.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <OGRE/OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>


namespace drone_panel{

    DronePanel::DronePanel(QWidget* parent): rviz::Panel(parent){
        cf = 0; //control frame is originally global robot
        mapping = false;

        // Initialize publishers
        cam_pos_pub = n.advertise<geometry_msgs::Point>("rviz_camera_p", 1);
        quat_pub = n.advertise<geometry_msgs::Quaternion>("rviz_camera_q", 1);
        cf_pub = n.advertise<std_msgs::String>("/control_frame", 1);
        mapping_pub = n.advertise<std_msgs::String>("mappingToggle", 1);

        battery_sub = n.subscribe("/tello/battery", 1, &DronePanel::batteryCallback, this);

        QWidget *cfBox = new QWidget;
        QHBoxLayout* cfLayout = new QHBoxLayout(cfBox);
        cfBox->setStyleSheet("background-color: #dae3e3; border-radius: 10px; border-color: #b6b8b8");
        cfBox->setFixedWidth(500);
        QPushButton* toggleControlbutton = new QPushButton("Toggle Control Frame");
        toggleControlbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2px; border-radius: 10px; border-color: #B6D5E7; font: bold 20px; min-width: 10em; padding: 6px;");
        QLabel* curr_cf = new QLabel("panda_link0");
        curr_cf->setAlignment(Qt::AlignCenter);
        curr_cf->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Fixed);
        QFont curr_cf_font = curr_cf -> font();
        curr_cf_font.setPointSize(18);
        curr_cf_font.setBold(true);
        curr_cf->setFont(curr_cf_font);
        cfLayout->addWidget(curr_cf);
        cfLayout->addWidget(toggleControlbutton);

        QWidget *mappingBox = new QWidget;
        QHBoxLayout* mappingLayout = new QHBoxLayout(mappingBox);
        mappingBox->setStyleSheet("background-color: #dae3e3; border-radius: 10px; border-color: #b6b8b8");
        mappingBox->setFixedWidth(300);
        QPushButton* toggleMappingbutton = new QPushButton("Start Mapping");
        toggleMappingbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2px; border-radius: 10px; border-color: #B6D5E7; font: bold 20px; min-width: 10em; padding: 6px;");
        mappingLayout->addWidget(toggleMappingbutton);

        QWidget *batteryBox = new QWidget;
        QHBoxLayout* batteryLayout = new QHBoxLayout(batteryBox);
        batteryBox->setStyleSheet("background-color: #dae3e3; border-radius: 10px; border-color: #b6b8b8");
        batteryBox->setFixedWidth(300);
        bat = new QLabel("Drone Battery: NA");
        bat->setAlignment(Qt::AlignCenter);
        bat->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Fixed);
        QFont bat_font = curr_cf -> font();
        bat_font.setPointSize(18);
        bat_font.setBold(true);
        bat->setFont(bat_font);
        batteryLayout->addWidget(bat);

        QHBoxLayout* hlayout = new QHBoxLayout;
        hlayout->addWidget(cfBox);
        hlayout->addWidget(mappingBox);
        hlayout->addWidget(batteryBox);
        hlayout->setSpacing(0);
        setLayout(hlayout);

        // Start trial sends a trigger
        connect(toggleControlbutton, &QPushButton::clicked, [this,toggleControlbutton,curr_cf](){
           if(cf==0){
               this->cf++;
               s_out.data ="drone";
               curr_cf->setText("drone");
           }
           else if(cf==1){
               this->cf++;
               s_out.data ="panda_gripper";
               curr_cf->setText("panda_gripper");
           }
           else{
               this->cf = 0;
               s_out.data ="panda_link0";
               curr_cf->setText("panda_link0");
           }
           cf_pub.publish(s_out);
        });

        // Start trial sends a trigger
        connect(toggleMappingbutton, &QPushButton::clicked, [this,toggleMappingbutton](){
           if(!mapping){
            s_out.data = "on";
            toggleMappingbutton->setText("End Mapping");
            toggleMappingbutton->setStyleSheet("background-color: #FF968A; border-style: solid; border-width: 2px; border-radius: 10px; border-color: #B6D5E7; font: bold 20px; min-width: 10em; padding: 6px;");
            mapping = true;
           }
           else{
               s_out.data = "off";
               toggleMappingbutton->setText("Start Mapping");
               toggleMappingbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2px; border-radius: 10px; border-color: #B6D5E7; font: bold 20px; min-width: 10em; padding: 6px;");
               mapping = false;
           }
           mapping_pub.publish(s_out);
        });

        
        // Timer used to publish the camera orientation from RVIZ for camera-centric controls
        QTimer* output_timer = new QTimer( this );  
        connect(output_timer, &QTimer::timeout, [this](){
            rviz::ViewManager* viewm = vis_manager_->getViewManager();
            rviz::ViewController* vc = viewm->getCurrent();
            Ogre::Camera* camera = vc->getCamera();
            const Ogre::Quaternion quat = camera->getOrientation();
            const Ogre::Vector3 cam_pos = camera->getPosition();
            
            // Convert from Ogre to ROS message
            q_out.x = quat.x; q_out.y = quat.y; q_out.z = quat.z; q_out.w = quat.w;
            pos_out.x = cam_pos.x; pos_out.y = cam_pos.y; pos_out.z = cam_pos.z; 
            quat_pub.publish(q_out);
            cam_pos_pub.publish(pos_out);
        }); 
        output_timer->start(100);

    }

    void DronePanel::batteryCallback(std_msgs::Int16 data){
        // Update the label with battery
        bat->setText(("Drone Battery: "+std::to_string(data.data)+"%").c_str());
    }

} // end namespace

// Make pluginlib aware of the class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(drone_panel::DronePanel,rviz::Panel)
