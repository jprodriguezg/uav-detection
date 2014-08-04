
(cl:in-package :asdf)

(defsystem "drone_GPS-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GPS_data" :depends-on ("_package_GPS_data"))
    (:file "_package_GPS_data" :depends-on ("_package"))
  ))