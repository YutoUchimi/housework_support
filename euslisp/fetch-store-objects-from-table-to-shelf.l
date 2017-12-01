#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "store_objects_from_table_to_shelf" :anonymous nil)

(require "package://fetcheus/fetch-interface.l")
(require "models/room73b2-scene.l")


(defvar obj-list)
(defvar place-pos-fetch-frame)
(defvar room-to-map-transform)

(setq place-pos-fetch-frame (float-vector 800 0 1500))
(setq room-to-map-transform
      (make-coords :pos (float-vector 0 6675 0) :rpy (float-vector 0 0 0)))


(defun init (&key (moveit nil))
  (unless (boundp '*ri*)
    (fetch-init))
  (when moveit
    (unless (boundp '*co*)
      (setq *co* (instance collision-object-publisher :init))))

  (unless (boundp '*room*)
    (setq *room* (room73b2)))

  (send *fetch* :move-to (make-coords) :world)
  (send *room* :move-to room-to-map-transform :world)

  (setq obj-list (list *fetch* *room*))
  (objects obj-list)

  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)
  )


;; TODO
(defun make-collision-object ()
  (when (boundp '*co*)
    (send *co* :add-object (send *room* :object "room73b2-coe-450-shelf")
          :frame-id "base_link"
          :relative-pose (send (send *room* :object "room73b2-coe-450-shelf") :copy-worldcoords)
          :object-id (format nil "table"))

    (send *co* :add-object (send *room* :object "room73b2-kitchen")
          :frame-id "base_link"
          :relative-pose (send (send *room* :object "room73b2-kitchen") :copy-worldcoords)
          :object-id (format nil "kitchen"))

    (send *co* :add-object (send *room* :object "room73b2-kitchen-shelf")
          :frame-id "base_link"
          :relative-pose (send (send *room* :object "room73b2-kitchen-shelf") :copy-worldcoords)
          :object-id (format nil "kitchen-shelf"))
    ))


(defun go-to-table ()
  (send *fetch* :move-to
        (send (send *room* :spot "coe-spot") :copy-worldcoords)
        :world)
  (send *irtviewer* :redraw)
  (send *ri* :move-to
        (send (send *room* :spot "coe-spot") :copy-worldcoords)
        :frame-id "/map"
        :no-wait nil)
  )


(defun recognize-object (bbox-x bbox-y bbox-z)
  (let (bboxes-topic bboxes-msg bbox
        centroids-topic centroids-msg centroid)
    ;; look down
    (send *fetch* :head :neck-p :joint-angle (deg2rad 15))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)

    (setq bboxes-topic "cluster_indices_decomposer_target/boxes")
    (setq centroids-topic "cluster_indices_decomposer_target/centroid_pose_array")

    ;; subscribe topics
    (ros::ros-info "Wait for subscribing: ~A~%" bboxes-topic)
    (ros::ros-info "Wait for subscribing: ~A~%" centroids-topic)
    (setq bboxes-msg
      (one-shot-subscribe bboxes-topic
                          jsk_recognition_msgs::BoundingBoxArray))
    (setq centroids-msg
      (one-shot-subscribe centroids-topic
                          geometry_msgs::PoseArray))

    (setq bbox (car (send bboxes-msg :boxes)))
    (setq centroid (car (send centroids-msg :poses)))

    ;; get position of bbox (frame: base_link)
    ;; convert metric: m -> mm
    (setq bbox-x (* 1000 (send centroid :position :x)))
    (setq bbox-y (* 1000 (send centroid :position :y)))
    (setq bbox-z (* 1000 (send bbox :pose :position :z)))
    (ros::ros-info "bbox-x: ~A, bbox-y: ~A, bbox-z: ~A~%" bbox-x bbox-y bbox-z)
    ))


(defun pick-object (bbox-x bbox-y bbox-z)
  (let ((bbox-pos-fetch-frame (float-vector bbox-x bbox-y bbox-z)))
    ;; approach to in front of the object
    (send *fetch* :inverse-kinematics
          (make-coords :pos (v+ bbox-pos-fetch-frame (float-vector -100 0 0))
                       :rpy (float-vector 0 0 0))
          :debug-view nil
          :revert-if-fail t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
    (send *ri* :wait-interpolation)

    ;; approach to the very place of the object
    (send *fetch* :inverse-kinematics
          (make-coords :pos bbox-pos-fetch-frame
                       :rpy (float-vector 0 0 0))
          :debug-view nil
          :revert-if-fail t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    ;; grasp object
    (send *ri* :go-grasp :effort 50)

    ;; lift up the object
    (send *fetch* :inverse-kinematics
          (make-coords :pos (v+ bbox-pos-fetch-frame (float-vector 0 0 200))
                       :rpy (float-vector 0 0 0))
          :debug-view nil
          :revert-if-fail t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)

    ;; reset pose with object held in robot's hand
    (send *fetch* :reset-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
    (send *ri* :wait-interpolation)
    ))


(defun go-to-shelf ()
  (send *fetch* :move-to
        (send (send *room* :spot "/eng2/7f/room73B2-front-of-kitchenboard") :copy-worldcoords)
        :world)
  (send *irtviewer* :redraw)
  (send *ri* :move-to
        (send (send *room* :spot "/eng2/7f/room73B2-front-of-kitchenboard") :copy-worldcoords)
        :frame-id "/map"
        :no-wait nil)
  )


(defun place-object ()
  ;; bring object to in front of the shelf
  (send *fetch* :inverse-kinematics
        (make-coords :pos (v+ place-pos-fetch-frame (float-vector -200 0 100))
                     :rpy (float-vector 0 0 0))
        :debug-view nil
        :revert-if-fail t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)

  ;; bring object to over the target place
  (send *fetch* :inverse-kinematics
        (make-coords :pos (v+ place-pos-fetch-frame (float-vector 0 0 100))
                     :rpy (float-vector 0 0 0))
        :debug-view nil
        :revert-if-fail t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; bring object to the very target place
  (send *fetch* :inverse-kinematics
        (make-coords :pos place-pos-fetch-frame
                     :rpy (float-vector 0 0 0))
        :debug-view nil
        :revert-if-fail t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; release object
  (send *ri* stop-grasp)

  ;; reset pose
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)
  )


(defun main ()
  (let (bbox-x bbox-y bbox-z)
    (init)
    (make-collision-object)
    (go-to-table)
    (recognize-object bbox-x bbox-y bbox-z)
    (pick-object bbox-x bbox-y bbox-z)
    (go-to-shelf)
    (place-object)
    ))