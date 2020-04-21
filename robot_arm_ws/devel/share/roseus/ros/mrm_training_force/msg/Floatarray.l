;; Auto-generated. Do not edit!


(when (boundp 'mrm_training_force::Floatarray)
  (if (not (find-package "MRM_TRAINING_FORCE"))
    (make-package "MRM_TRAINING_FORCE"))
  (shadow 'Floatarray (find-package "MRM_TRAINING_FORCE")))
(unless (find-package "MRM_TRAINING_FORCE::FLOATARRAY")
  (make-package "MRM_TRAINING_FORCE::FLOATARRAY"))

(in-package "ROS")
;;//! \htmlinclude Floatarray.msg.html


(defclass mrm_training_force::Floatarray
  :super ros::object
  :slots (_data ))

(defmethod mrm_training_force::Floatarray
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mrm_training_force::Floatarray :md5sum-) "788898178a3da2c3718461eecda8f714")
(setf (get mrm_training_force::Floatarray :datatype-) "mrm_training_force/Floatarray")
(setf (get mrm_training_force::Floatarray :definition-)
      "float64[] data

")



(provide :mrm_training_force/Floatarray "788898178a3da2c3718461eecda8f714")


