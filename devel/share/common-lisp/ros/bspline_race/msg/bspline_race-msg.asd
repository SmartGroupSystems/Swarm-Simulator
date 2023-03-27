
(cl:in-package :asdf)

(defsystem "bspline_race-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BsplineTraj" :depends-on ("_package_BsplineTraj"))
    (:file "_package_BsplineTraj" :depends-on ("_package"))
  ))