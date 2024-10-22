
(cl:in-package :asdf)

(defsystem "navigation2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Enc_dist" :depends-on ("_package_Enc_dist"))
    (:file "_package_Enc_dist" :depends-on ("_package"))
    (:file "Goal" :depends-on ("_package_Goal"))
    (:file "_package_Goal" :depends-on ("_package"))
    (:file "Planner_state" :depends-on ("_package_Planner_state"))
    (:file "_package_Planner_state" :depends-on ("_package"))
    (:file "auto" :depends-on ("_package_auto"))
    (:file "_package_auto" :depends-on ("_package"))
    (:file "detection" :depends-on ("_package_detection"))
    (:file "_package_detection" :depends-on ("_package"))
    (:file "enc_feed" :depends-on ("_package_enc_feed"))
    (:file "_package_enc_feed" :depends-on ("_package"))
    (:file "gps_data" :depends-on ("_package_gps_data"))
    (:file "_package_gps_data" :depends-on ("_package"))
    (:file "imu_angle" :depends-on ("_package_imu_angle"))
    (:file "_package_imu_angle" :depends-on ("_package"))
    (:file "red" :depends-on ("_package_red"))
    (:file "_package_red" :depends-on ("_package"))
  ))