
(in-package :asdf)

(defsystem "mic_array-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "MicArrayParam" :depends-on ("_package"))
    (:file "_package_MicArrayParam" :depends-on ("_package"))
    ))
