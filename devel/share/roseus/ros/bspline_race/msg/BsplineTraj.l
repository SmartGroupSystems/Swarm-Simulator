;; Auto-generated. Do not edit!


(when (boundp 'bspline_race::BsplineTraj)
  (if (not (find-package "BSPLINE_RACE"))
    (make-package "BSPLINE_RACE"))
  (shadow 'BsplineTraj (find-package "BSPLINE_RACE")))
(unless (find-package "BSPLINE_RACE::BSPLINETRAJ")
  (make-package "BSPLINE_RACE::BSPLINETRAJ"))

(in-package "ROS")
;;//! \htmlinclude BsplineTraj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bspline_race::BsplineTraj
  :super ros::object
  :slots (_header _position _velocity _acceleration _yaw _yaw_rate _current_seq ))

(defmethod bspline_race::BsplineTraj
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) ())
    ((:velocity __velocity) ())
    ((:acceleration __acceleration) ())
    ((:yaw __yaw) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:current_seq __current_seq) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   (setq _yaw (float __yaw))
   (setq _yaw_rate (float __yaw_rate))
   (setq _current_seq (round __current_seq))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:current_seq
   (&optional __current_seq)
   (if __current_seq (setq _current_seq __current_seq)) _current_seq)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/PoseStamped[] _position
    (apply #'+ (send-all _position :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _velocity
    (apply #'+ (send-all _velocity :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _acceleration
    (apply #'+ (send-all _acceleration :serialization-length)) 4
    ;; float32 _yaw
    4
    ;; float32 _yaw_rate
    4
    ;; uint32 _current_seq
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/PoseStamped[] _position
     (write-long (length _position) s)
     (dolist (elem _position)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _velocity
     (write-long (length _velocity) s)
     (dolist (elem _velocity)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _acceleration
     (write-long (length _acceleration) s)
     (dolist (elem _acceleration)
       (send elem :serialize s)
       )
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _current_seq
       (write-long _current_seq s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/PoseStamped[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _position)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _velocity)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _acceleration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _acceleration (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _acceleration)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _current_seq
     (setq _current_seq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bspline_race::BsplineTraj :md5sum-) "47fe784cc6ad92a6cd7d1488e4d545d9")
(setf (get bspline_race::BsplineTraj :datatype-) "bspline_race/BsplineTraj")
(setf (get bspline_race::BsplineTraj :definition-)
      "std_msgs/Header header

geometry_msgs/PoseStamped[] position
geometry_msgs/PoseStamped[] velocity
geometry_msgs/PoseStamped[] acceleration

float32 yaw
float32 yaw_rate
uint32 current_seq

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :bspline_race/BsplineTraj "47fe784cc6ad92a6cd7d1488e4d545d9")


