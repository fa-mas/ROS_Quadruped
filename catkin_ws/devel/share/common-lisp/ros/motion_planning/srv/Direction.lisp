; Auto-generated. Do not edit!


(cl:in-package motion_planning-srv)


;//! \htmlinclude Direction-request.msg.html

(cl:defclass <Direction-request> (roslisp-msg-protocol:ros-message)
  ((ready
    :reader ready
    :initarg :ready
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Direction-request (<Direction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Direction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Direction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_planning-srv:<Direction-request> is deprecated: use motion_planning-srv:Direction-request instead.")))

(cl:ensure-generic-function 'ready-val :lambda-list '(m))
(cl:defmethod ready-val ((m <Direction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_planning-srv:ready-val is deprecated.  Use motion_planning-srv:ready instead.")
  (ready m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Direction-request>) ostream)
  "Serializes a message object of type '<Direction-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ready) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Direction-request>) istream)
  "Deserializes a message object of type '<Direction-request>"
    (cl:setf (cl:slot-value msg 'ready) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Direction-request>)))
  "Returns string type for a service object of type '<Direction-request>"
  "motion_planning/DirectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Direction-request)))
  "Returns string type for a service object of type 'Direction-request"
  "motion_planning/DirectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Direction-request>)))
  "Returns md5sum for a message object of type '<Direction-request>"
  "91ac0cf00c7cda6ec66ed487aa4d6933")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Direction-request)))
  "Returns md5sum for a message object of type 'Direction-request"
  "91ac0cf00c7cda6ec66ed487aa4d6933")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Direction-request>)))
  "Returns full string definition for message of type '<Direction-request>"
  (cl:format cl:nil "bool ready~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Direction-request)))
  "Returns full string definition for message of type 'Direction-request"
  (cl:format cl:nil "bool ready~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Direction-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Direction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Direction-request
    (cl:cons ':ready (ready msg))
))
;//! \htmlinclude Direction-response.msg.html

(cl:defclass <Direction-response> (roslisp-msg-protocol:ros-message)
  ((dir
    :reader dir
    :initarg :dir
    :type cl:string
    :initform "")
   (ang
    :reader ang
    :initarg :ang
    :type cl:float
    :initform 0.0)
   (vec
    :reader vec
    :initarg :vec
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Direction-response (<Direction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Direction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Direction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_planning-srv:<Direction-response> is deprecated: use motion_planning-srv:Direction-response instead.")))

(cl:ensure-generic-function 'dir-val :lambda-list '(m))
(cl:defmethod dir-val ((m <Direction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_planning-srv:dir-val is deprecated.  Use motion_planning-srv:dir instead.")
  (dir m))

(cl:ensure-generic-function 'ang-val :lambda-list '(m))
(cl:defmethod ang-val ((m <Direction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_planning-srv:ang-val is deprecated.  Use motion_planning-srv:ang instead.")
  (ang m))

(cl:ensure-generic-function 'vec-val :lambda-list '(m))
(cl:defmethod vec-val ((m <Direction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_planning-srv:vec-val is deprecated.  Use motion_planning-srv:vec instead.")
  (vec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Direction-response>) ostream)
  "Serializes a message object of type '<Direction-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dir))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ang))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'vec))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Direction-response>) istream)
  "Deserializes a message object of type '<Direction-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dir) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dir) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ang) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'vec) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'vec)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Direction-response>)))
  "Returns string type for a service object of type '<Direction-response>"
  "motion_planning/DirectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Direction-response)))
  "Returns string type for a service object of type 'Direction-response"
  "motion_planning/DirectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Direction-response>)))
  "Returns md5sum for a message object of type '<Direction-response>"
  "91ac0cf00c7cda6ec66ed487aa4d6933")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Direction-response)))
  "Returns md5sum for a message object of type 'Direction-response"
  "91ac0cf00c7cda6ec66ed487aa4d6933")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Direction-response>)))
  "Returns full string definition for message of type '<Direction-response>"
  (cl:format cl:nil "string dir~%float32 ang~%float32[3] vec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Direction-response)))
  "Returns full string definition for message of type 'Direction-response"
  (cl:format cl:nil "string dir~%float32 ang~%float32[3] vec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Direction-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'dir))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vec) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Direction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Direction-response
    (cl:cons ':dir (dir msg))
    (cl:cons ':ang (ang msg))
    (cl:cons ':vec (vec msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Direction)))
  'Direction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Direction)))
  'Direction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Direction)))
  "Returns string type for a service object of type '<Direction>"
  "motion_planning/Direction")