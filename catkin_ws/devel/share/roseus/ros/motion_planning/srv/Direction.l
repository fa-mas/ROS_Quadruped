;; Auto-generated. Do not edit!


(when (boundp 'motion_planning::Direction)
  (if (not (find-package "MOTION_PLANNING"))
    (make-package "MOTION_PLANNING"))
  (shadow 'Direction (find-package "MOTION_PLANNING")))
(unless (find-package "MOTION_PLANNING::DIRECTION")
  (make-package "MOTION_PLANNING::DIRECTION"))
(unless (find-package "MOTION_PLANNING::DIRECTIONREQUEST")
  (make-package "MOTION_PLANNING::DIRECTIONREQUEST"))
(unless (find-package "MOTION_PLANNING::DIRECTIONRESPONSE")
  (make-package "MOTION_PLANNING::DIRECTIONRESPONSE"))

(in-package "ROS")





(defclass motion_planning::DirectionRequest
  :super ros::object
  :slots (_ready ))

(defmethod motion_planning::DirectionRequest
  (:init
   (&key
    ((:ready __ready) nil)
    )
   (send-super :init)
   (setq _ready __ready)
   self)
  (:ready
   (&optional (__ready :null))
   (if (not (eq __ready :null)) (setq _ready __ready)) _ready)
  (:serialization-length
   ()
   (+
    ;; bool _ready
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ready
       (if _ready (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ready
     (setq _ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass motion_planning::DirectionResponse
  :super ros::object
  :slots (_dir _ang _vec ))

(defmethod motion_planning::DirectionResponse
  (:init
   (&key
    ((:dir __dir) "")
    ((:ang __ang) 0.0)
    ((:vec __vec) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _dir (string __dir))
   (setq _ang (float __ang))
   (setq _vec __vec)
   self)
  (:dir
   (&optional __dir)
   (if __dir (setq _dir __dir)) _dir)
  (:ang
   (&optional __ang)
   (if __ang (setq _ang __ang)) _ang)
  (:vec
   (&optional __vec)
   (if __vec (setq _vec __vec)) _vec)
  (:serialization-length
   ()
   (+
    ;; string _dir
    4 (length _dir)
    ;; float32 _ang
    4
    ;; float32[3] _vec
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _dir
       (write-long (length _dir) s) (princ _dir s)
     ;; float32 _ang
       (sys::poke _ang (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[3] _vec
     (dotimes (i 3)
       (sys::poke (elt _vec i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _dir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _ang
     (setq _ang (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[3] _vec
   (dotimes (i (length _vec))
     (setf (elt _vec i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(defclass motion_planning::Direction
  :super ros::object
  :slots ())

(setf (get motion_planning::Direction :md5sum-) "91ac0cf00c7cda6ec66ed487aa4d6933")
(setf (get motion_planning::Direction :datatype-) "motion_planning/Direction")
(setf (get motion_planning::Direction :request) motion_planning::DirectionRequest)
(setf (get motion_planning::Direction :response) motion_planning::DirectionResponse)

(defmethod motion_planning::DirectionRequest
  (:response () (instance motion_planning::DirectionResponse :init)))

(setf (get motion_planning::DirectionRequest :md5sum-) "91ac0cf00c7cda6ec66ed487aa4d6933")
(setf (get motion_planning::DirectionRequest :datatype-) "motion_planning/DirectionRequest")
(setf (get motion_planning::DirectionRequest :definition-)
      "bool ready
---
string dir
float32 ang
float32[3] vec
")

(setf (get motion_planning::DirectionResponse :md5sum-) "91ac0cf00c7cda6ec66ed487aa4d6933")
(setf (get motion_planning::DirectionResponse :datatype-) "motion_planning/DirectionResponse")
(setf (get motion_planning::DirectionResponse :definition-)
      "bool ready
---
string dir
float32 ang
float32[3] vec
")



(provide :motion_planning/Direction "91ac0cf00c7cda6ec66ed487aa4d6933")


