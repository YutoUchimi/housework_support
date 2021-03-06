#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_gui_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "store_objects_from_table_to_shelf" :anonymous nil)

(require "package://fetcheus/fetch-interface.l")
(require "models/room73b2-scene.l")


(defvar place-coords-world-frame)
(defvar room-to-map-transform)
(defvar shelf-spot)
(defvar show-irtviewer)

;; Parameters
;; >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
(setq room-to-map-transform
      (make-coords :pos (float-vector 0 6675 0) :rpy (float-vector 0 0 0)))
;; <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


(defun get-map-to-base-coords ()
  (let (transform-topic (transform-msg nil)
        pos-x pos-y pos-z
        rot-x rot-y rot-z rot-w rot-rpy)
    (setq transform-topic "tf_to_transform/output")

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
           (flatten (send-all (send *fetch* :bodies) :vertices))))))


(defun init (&key (moveit nil))
  (ros::ros-info "Initializing...")
  (unless (boundp '*ri*)
    (fetch-init))
  (when moveit
    (unless (boundp '*co*)
      (setq *co* (instance collision-object-publisher :init))))
  (unless (boundp '*room*)
    (setq *room* (room73b2)))

  (send *fetch* :move-to (get-map-to-base-coords) :world)
  (send *room* :move-to room-to-map-transform :world)

  (setq show-irtviewer (ros::get-param "~show_irtviewer"))
  (when show-irtviewer
    (objects (list *fetch* *room*)))

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

  (send *fetch* :reset-pose)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)
  )


(defun make-collision-object ()
  (when (boundp '*co*)
    (ros::ros-info "Adding collision object...")
    (send *co* :add-object (send *room* :object "room73b2-coe-450-shelf")
          :frame-id "base_link"
          :relative-pose (send (send *room* :object "room73b2-coe-450-shelf") :copy-worldcoords)
          :object-id (format nil "table"))

    (send *co* :add-object (send *room* :object "room73b2-coe-800-shelf")
          :frame-id "base_link"
          :relative-pose (send (send *room* :object "room73b2-coe-800-shelf") :copy-worldcoords)
          :object-id (format nil "side-table"))

    (send *co* :add-object (send *room* :object "room73b2-toshiba-clacio-refrigerator")
          :frame-id "base_link"
          :relative-pose (send (send *room* :object "room73b2-toshiba-clacio-refrigerator") :copy-worldcoords)
          :object-id (format nil "refrigerator"))

    (send *co* :add-object (send *room* :object "room73b2-kitchen")
          :frame-id "base_link"
          :relative-pose (send (send *room* :object "room73b2-kitchen") :copy-worldcoords)
          :object-id (format nil "kitchen"))

    (send *co* :add-object (send *room* :object "room73b2-kitchen-shelf")
          :frame-id "base_link"
          :relative-pose (send (send *room* :object "room73b2-kitchen-shelf") :copy-worldcoords)
          :object-id (format nil "kitchen-shelf"))
    ))


(defun wait-for-user-input ()
  (let ((req (instance jsk_gui_msgs::YesNoRequest :init))
        (res nil)
        (pressed-p nil))
    (ros::wait-for-service "/rviz/yes_no_button")
    (ros::ros-info "Please select Yes to start program.")
    ;; Wait until Yes is pressed
    (while (not pressed-p)
      (setq res (ros::service-call "/rviz/yes_no_button" req))
      (when (send res :yes)
        (ros::ros-info "Requesting to start program.")
        (setq pressed-p t)))
    ))


(defun go-to-table ()
  (ros::ros-info "Moving to table...")
  (send *fetch* :move-to
        (send (send *room* :spot "coe-spot") :copy-worldcoords)
        :world)
  (redraw)
  (send *ri* :move-to
        (send (send *room* :spot "coe-spot") :copy-worldcoords)
        :frame-id "/map"
        :no-wait nil)
  (send *fetch* :move-to (get-map-to-base-coords) :world)
  (redraw)
  )


(defun recognize-object (bbox-coords-world-frame)
  (let (fixed-frame
        bboxes-topic bboxes-msg (bbox nil)
        centroids-topic centroids-msg (centroid nil)
        bbox-x bbox-y bbox-z
        bbox-coords-fetch-frame)
    (ros::ros-info "Searching for object...")

    ;; Look down
    (send *fetch* :head :neck-p :joint-angle 15)
    (redraw)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)

    (setq bboxes-topic "cluster_indices_decomposer_target/boxes")
    (setq centroids-topic "cluster_indices_decomposer_target/centroid_pose_array")

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

      ;; Get largest bounding box
      (setq bbox (car (send bboxes-msg :boxes)))
      (setq centroid (car (send centroids-msg :poses)))
      )

    ;; Get position of bbox (frame: base_link)
    ;; Convert metric: m -> mm
    (setq bbox-x (* 1000 (send (send centroid :position) :x)))
    (setq bbox-y (* 1000 (send (send centroid :position) :y)))
    (setq bbox-z (* 1000 (send (send (send bbox :pose) :position) :z)))

    ;; Get rosparam from /fixed_frame
    (setq fixed-frame (ros::get-param "~fixed_frame"))

    ;; Get coordinates of bbox (frame: world)
    (setq bbox-coords-fetch-frame (make-cascoords))
    (send (send *fetch* :link (format nil "~a" fixed-frame)) :assoc bbox-coords-fetch-frame)
    (send bbox-coords-fetch-frame :newcoords
          :newpos (float-vector bbox-x bbox-y bbox-z))
    (setq bbox-coords-world-frame
          (send *room* :transformation bbox-coords-fetch-frame))
    (ros::ros-info "Bounding box found (at fetch frame): ~a" bbox-coords-fetch-frame)
    (ros::ros-info "Bounding box found (at world frame): ~a" bbox-coords-world-frame)
    ))


(defun pick-object (bbox-coords-world-frame)
  (ros::ros-info "Picking object...")

  ;; Approach to in front of the object
  (send *fetch* :inverse-kinematics
        (make-coords :pos (v+ (send bbox-coords-world-frame :copy-worldcoords)
                              (float-vector -100 0 50))
                     :rpy (float-vector 0 0 0))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)

  ;; Approach to the very place of the object
  (send *fetch* :inverse-kinematics
        (make-coords :pos (send bbox-coords-world-frame :copy-worldcoords)
                     :rpy (float-vector 0 0 0))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; Grasp object
  (ros::ros-info "Start grasping object...")
  (send *ri* :go-grasp :effort 50)

  ;; Lift up the object
  (send *fetch* :inverse-kinematics
        (make-coords :pos (v+ (send bbox-coords-world-frame :copy-worldcoords)
                              (float-vector 0 0 200))
                     :rpy (float-vector 0 0 0))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Reset pose with object held in robot's hand
  (send *fetch* :reset-pose)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)
  )


(defun go-to-shelf ()
  (ros::ros-info "Moving to shelf...")
  (send *fetch* :move-to shelf-spot
        :world)
  (redraw)
  (send *ri* :move-to shelf-spot
        :frame-id "/map"
        :no-wait nil)
  (send *fetch* :move-to (get-map-to-base-coords) :world)
  (redraw)
  )


(defun place-object ()
  (ros::ros-info "Placing object...")
  ;; Bring object to in front of the shelf
  (send *fetch* :inverse-kinematics
        (make-coords :pos (v+ (send place-coords-world-frame :pos) (float-vector 200 0 100))
                     :rpy (car (rpy-angle (send place-coords-world-frame :rot))))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)

  ;; Bring object to over the target place
  (send *fetch* :inverse-kinematics
        (make-coords :pos (v+ (send place-coords-world-frame :pos) (float-vector 50 0 50))
                     :rpy (car (rpy-angle (send place-coords-world-frame :rot))))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Bring object to the very target place
  (send *fetch* :inverse-kinematics place-coords-world-frame
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; Release object
  (ros::ros-info "Releasing object...")
  (send *ri* :stop-grasp)

  ;; Middle pose to reset pose
  (send *fetch* :inverse-kinematics
        (make-coords :pos (v+ (send place-coords-world-frame :pos) (float-vector 200 0 100))
                     :rpy (car (rpy-angle (send place-coords-world-frame :rot))))
        :stop 500
        :debug-view nil
        :revert-if-fail t)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Reset pose
  (send *fetch* :reset-pose)
  (redraw)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)
  )


(defun main ()
  (let (bbox-coords-world-frame)
    (init :moveit t)
    (make-collision-object)
    (wait-for-user-input)
    (go-to-table)
    (recognize-object bbox-coords-world-frame)
    (pick-object bbox-coords-world-frame)
    (go-to-shelf)
    (place-object)
    )
  t)
