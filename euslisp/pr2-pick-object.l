#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_gui_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "pr2_pick_object" :anonymous nil)

(require "package://pr2eus/pr2-interface.l")
(require "models/room73b2-scene.l")


;; Please remap these.
(defvar bboxes-topic "~bboxes")
(defvar centroids-topic "~centroid_poses")
(defvar user-input-service "~start_trigger")
(defvar transform-topic "~tf_map_to_base")

;; Global constants
(defvar room-to-map-transform
  (make-coords :pos (float-vector 0 6675 0) :rpy (float-vector 0 0 0)))

;; Global variables
(defvar place-coords-world-frame)
(defvar shelf-spot)
(defvar show-irtviewer)


(defun init ()
  (ros::ros-info "Initializing...")
  (unless (boundp '*ri*)
    (pr2-init nil))
  (unless (boundp '*room*)
    (setq *room* (room73b2)))

  (send *pr2* :move-to (get-map-to-base-coords) :world)
  (send *room* :move-to room-to-map-transform :world)

  (setq show-irtviewer (ros::get-param "~show_irtviewer"))
  (when show-irtviewer
    (objects (list *pr2* *room*)))

  ;; parameters
  ;; >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  (let (tmp-coords)
    (setq tmp-coords
          (send (send *room* :object "room73b2-kitchen-shelf") :copy-worldcoords))
    (setq place-coords-world-frame
          (make-coords :pos (v+ (send tmp-coords :pos)
                                (float-vector 100 0 80))
                       :rpy (float-vector pi 0 0)))

    (setq tmp-coords
          (send (send *room* :spot "/eng2/7f/room73B2-front-of-kitchenboard") :copy-worldcoords))
    (setq shelf-spot
          (make-coords :pos (v+ (send tmp-coords :pos)
                                (float-vector -200 0 0))
                       :rpy (float-vector pi 0 0)))
    )
  ;; <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  (pr2-tuckarm-pose)
  (redraw)
  )


(defun get-map-to-base-coords ()
  (let ((transform-msg nil)
        pos-x pos-y pos-z
        rot-x rot-y rot-z rot-w rot-rpy)

    ;; Subscribe tf_to_transform/output
    (while (null transform-msg)
      (ros::ros-info "Wait for subscribing: ~A~%" transform-topic)
      (setq transform-msg
            (one-shot-subscribe transform-topic
                                geometry_msgs::TransformStamped))
      )

    ;; Convert metric for eus: m -> mm
    (setq pos-x (* (send (send (send transform-msg :transform) :translation) :x) 1000))
    (setq pos-y (* (send (send (send transform-msg :transform) :translation) :y) 1000))
    (setq pos-z (* (send (send (send transform-msg :transform) :translation) :z) 1000))

    ;; Change order for tf->eus: (x y z w) -> (w x y z)
    (setq rot-x (send (send (send transform-msg :transform) :rotation) :x))
    (setq rot-y (send (send (send transform-msg :transform) :rotation) :y))
    (setq rot-z (send (send (send transform-msg :transform) :rotation) :z))
    (setq rot-w (send (send (send transform-msg :transform) :rotation) :w))
    (setq rot-rpy (car (rpy-angle (quaternion2matrix (float-vector rot-w rot-x rot-y rot-z)))))

    (make-coords :pos (float-vector pos-x pos-y pos-z) :rpy rot-rpy)
    ))


(defun redraw ()
  (when show-irtviewer
    (send *irtviewer* :redraw)
    (send *irtviewer* :look-all
          (geo::make-bounding-box
           (flatten (send-all (send *pr2* :bodies) :vertices))))))


(defun wait-for-user-input ()
  (let ((req (instance jsk_gui_msgs::YesNoRequest :init))
        (res nil)
        (pressed-p nil))
    (ros::wait-for-service user-input-service)
    (ros::ros-info "Please select Yes to start program.")
    ;; Wait until Yes is pressed
    (while (not pressed-p)
      (setq res (ros::service-call user-input-service req))
      (when (send res :yes)
        (ros::ros-info "Requesting to start program.")
        (setq pressed-p t)))
    ))


(defun go-to-table ()
  (ros::ros-info "Moving to table...")
  (send *pr2* :move-to
        (send (send *room* :spot "coe-spot") :copy-worldcoords)
        :world)
  (redraw)
  (send *ri* :move-to
        (send (send *room* :spot "coe-spot") :copy-worldcoords)
        :frame-id "/map"
        :no-wait nil)
  (send *pr2* :move-to (get-map-to-base-coords) :world)
  (redraw)
  )


(defun recognize-object (bbox-coords-world-frame)
  (let (fixed-frame
        bboxes-msg (bbox nil)
        centroids-msg (centroid nil)
        bbox-x bbox-y bbox-z
        bbox-coords-pr2-fixed-frame)
    (ros::ros-info "Searching for object...")

    ;; Look down
    (send *pr2* :head :neck-p :joint-angle 15)
    (redraw)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)

    (while (or (null bbox) (null centroid))
      ;; Subscribe bbox & centroid topics
      (ros::ros-info "Wait for subscribing: ~A~%" bboxes-topic)
      (setq bboxes-msg
            (one-shot-subscribe bboxes-topic
                                jsk_recognition_msgs::BoundingBoxArray))
      (ros::ros-info "Wait for subscribing: ~A~%" centroids-topic)
      (setq centroids-msg
            (one-shot-subscribe centroids-topic
                                geometry_msgs::PoseArray))

      ;; Get first bounding box (assumed to be largest)
      (setq bbox (car (send bboxes-msg :boxes)))
      (setq centroid (car (send centroids-msg :poses)))
      )

    ;; Get rosparam from ~fixed_frame (/base_link?)
    (setq fixed-frame (ros::get-param "~fixed_frame"))

    ;; Get position of bbox (frame: ~fixed_frame)
    ;; Convert metric: m -> mm
    (setq bbox-x (* 1000 (send (send centroid :position) :x)))
    (setq bbox-y (* 1000 (send (send centroid :position) :y)))
    (setq bbox-z (* 1000 (send (send (send bbox :pose) :position) :z)))
    (setq bbox-coords-pr2-fixed-frame
          (make-cascoords :pos (float-vector bbox-x bbox-y bbox-z)))
    (send (send *pr2* :link (format nil "~a" fixed-frame))
          :assoc bbox-coords-pr2-fixed-frame)

    ;; Get coordinates of bbox (frame: world)
    (setq bbox-coords-world-frame
          (send *room* :transformation bbox-coords-pr2-fixed-frame))

    (ros::ros-info "Bounding box found (at PR2 frame)  : ~a" bbox-coords-pr2-fixed-frame)
    (ros::ros-info "Bounding box found (at world frame): ~a" bbox-coords-world-frame)
    ))


(defun pick-object (bbox-coords-world-frame)
  (ros::ros-info "Picking object...")

  ;; Approach to in front of the object
  (send *pr2* :inverse-kinematics
        (make-coords :pos (v+ (send bbox-coords-world-frame :copy-worldcoords)
                              (float-vector -100 0 50))
                     :rpy (float-vector 0 0 0))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;; Approach to the very place of the object
  (send *pr2* :inverse-kinematics
        (make-coords :pos (send bbox-coords-world-frame :copy-worldcoords)
                     :rpy (float-vector 0 0 0))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; Grasp object
  (ros::ros-info "Start grasping object...")
  (send *ri* :go-grasp :effort 50)

  ;; Lift up the object
  (send *pr2* :inverse-kinematics
        (make-coords :pos (v+ (send bbox-coords-world-frame :copy-worldcoords)
                              (float-vector 0 0 200))
                     :rpy (float-vector 0 0 0))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Tuck arm pose with object held in robot's hand
  (pr2-tuckarm-pose)
  (redraw)
  )


(defun go-to-shelf ()
  (ros::ros-info "Moving to shelf...")
  (send *pr2* :move-to shelf-spot
        :world)
  (redraw)
  (send *ri* :move-to shelf-spot
        :frame-id "/map"
        :no-wait nil)
  (send *pr2* :move-to (get-map-to-base-coords) :world)
  (redraw)
  )


(defun place-object ()
  (ros::ros-info "Placing object...")
  ;; Bring object to in front of the shelf
  (send *pr2* :inverse-kinematics
        (make-coords :pos (v+ (send place-coords-world-frame :pos) (float-vector 200 0 100))
                     :rpy (car (rpy-angle (send place-coords-world-frame :rot))))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  ;; Bring object to over the target place
  (send *pr2* :inverse-kinematics
        (make-coords :pos (v+ (send place-coords-world-frame :pos) (float-vector 50 0 50))
                     :rpy (car (rpy-angle (send place-coords-world-frame :rot))))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Bring object to the very target place
  (send *pr2* :inverse-kinematics place-coords-world-frame
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; Release object
  (ros::ros-info "Releasing object...")
  (send *ri* :stop-grasp)

  ;; Middle pose to tuck arm pose
  (send *pr2* :inverse-kinematics
        (make-coords :pos (v+ (send place-coords-world-frame :pos) (float-vector 200 0 100))
                     :rpy (car (rpy-angle (send place-coords-world-frame :rot))))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Tuck arm pose
  (pr2-tuckarm-pose)
  (redraw)
  )


(defun main ()
  (let (bbox-coords-world-frame)
    (init)
    (wait-for-user-input)
    (go-to-table)
    (recognize-object bbox-coords-world-frame)
    (pick-object bbox-coords-world-frame)
    (go-to-shelf)
    (place-object)
    )
  t)
