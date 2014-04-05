
(cl:in-package :asdf)

(defsystem "SpiderRobot_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "My2Num" :depends-on ("_package_My2Num"))
    (:file "_package_My2Num" :depends-on ("_package"))
    (:file "MyChar" :depends-on ("_package_MyChar"))
    (:file "_package_MyChar" :depends-on ("_package"))
    (:file "MyArray" :depends-on ("_package_MyArray"))
    (:file "_package_MyArray" :depends-on ("_package"))
  ))