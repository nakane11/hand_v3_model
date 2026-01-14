(load "../urdf/hand_v3.l")

(hand_v3_model)

(defclass marker
  :slots (robot-model marker-tb))

(defmethod marker
  (:init
    (robot)
    (setq robot-model robot)
    (send self :initialize-markers))

  (:get-link-by-name
    (name)
    (dolist (lk (send robot-model :links))
      (when (string-equal (send lk :name) name)
        (return-from :get-link-by-name lk)))
    nil)

  (:marker-object
    ()
    (let ((value-list (send marker-tb :list-values)))
      (nreverse (flatten value-list))))

  (:make-marker-body
    (pos color &key (size 6))
    (setq cyl (make-cylinder size 1))
    (send cyl :replace-pos pos)
    (send cyl :rotate (/ pi 2) :y)
    (send cyl :translate (float-vector 3 0 0) :world)
    (send cyl :set-color color)
    cyl)

  (:make-palm-marker-body
    (pos color)
    (setq cyl (make-cube 40 40 1))
    (send cyl :replace-pos pos)
    (send cyl :rotate (/ pi 2) :y)
    (send cyl :set-color color)
    cyl)

  (:add-marker
    (parent name pos mk-color &key (rot-axis nil) (rot-deg 0) (trans nil) (mk-size nil))
    (let ((mk (if mk-size
                (send self :make-marker-body pos mk-color :size mk-size)
                (send self :make-marker-body pos mk-color))))
      (when rot-axis
        (send mk :rotate (deg2rad rot-deg) rot-axis :world))
      (when trans
        (send mk :translate trans :world))
      (send parent :assoc mk)
      (sethash name marker-tb mk)
      ))

  (:add-palm-marker
    (parent name pos mk-color &key (rot-axis nil) (rot-deg 0) (trans nil) (mk-size nil))
    (let ((mk (if mk-size
                (send self :make-palm-marker-body pos mk-color :size mk-size)
                (send self :make-palm-marker-body pos mk-color))))
      (when rot-axis
        (send mk :rotate (deg2rad rot-deg) rot-axis :world))
      (when trans
        (send mk :translate trans :world))
      (send parent :assoc mk)
      (sethash name marker-tb mk)
      ))

  (:initialize-markers
    ()
    (setq marker-tb (make-hash-table :test #'string-equal))
    (setq front-color #f(1.0 0.2 0.2 0.6))
    (setq side-color #f(0.8 0.8 0.0 0.6))
    (dolist (finger (list "index" "middle" "ring" "little"))
      (let* ((l1 (send self :get-link-by-name (format nil "~A_link1" finger)))
             (l2 (send self :get-link-by-name (format nil "~A_link2" finger)))
             (l3 (send self :get-link-by-name (format nil "~A_link3" finger)))
             (p1 (scale 0.5 (v+ (send (send l1 :copy-worldcoords) :pos) (send (send l2 :copy-worldcoords) :pos))))
             (p2 (scale 0.5 (v+ (send (send l2 :copy-worldcoords) :pos) (send (send l3 :copy-worldcoords) :pos))))
             (p3 (send (send (send l3 :copy-worldcoords) :translate (float-vector -1.5 0 12) :world) :copy-pos)))
        (send self :add-marker l1 (send l1 :name) p1 front-color)
        (send self :add-marker l1 (format nil "~A-l" (send l1 :name)) p1 side-color
              :rot-axis :z :rot-deg -75 :trans (float-vector -5 -6.5 0))
        (send self :add-marker l1 (format nil "~A-r" (send l1 :name)) p1 side-color
              :rot-axis :z :rot-deg 75  :trans (float-vector -5 6.5 0))

        (send self :add-marker l2 (send l2 :name) p2 front-color)
        (send self :add-marker l2 (format nil "~A-l" (send l2 :name)) p2 side-color
              :rot-axis :z :rot-deg -75 :trans (float-vector -5 -6.5 0))
        (send self :add-marker l2 (format nil "~A-r" (send l2 :name)) p2 side-color
              :rot-axis :z :rot-deg 75  :trans (float-vector -5 6.5 0))

        (send self :add-marker l3 (send l3 :name) p3 front-color
              :rot-axis :y :rot-deg -12)
        ))

    (let* ((thumb-link0 (send self :get-link-by-name "thumb_link0"))
           (thumb-link1 (send self :get-link-by-name "thumb_link1"))
           (thumb-link2 (send self :get-link-by-name "thumb_link2"))
           (pos0 (send (send (send thumb-link0 :copy-worldcoords) :translate (float-vector -1.5 0 12) :world) :copy-pos))
           (pos1 (scale 0.5 (v+ (send (send thumb-link1 :copy-worldcoords) :pos) (send (send thumb-link2 :copy-worldcoords) :pos))))
           (pos2 (send (send (send thumb-link2 :copy-worldcoords) :translate (float-vector -1.5 0 12) :world) :copy-pos))
           (cyl0 (send self :make-marker-body pos0 front-color))
           (cyl1 (send self :make-marker-body pos1 front-color))
           (cyl2 (send self :make-marker-body pos2 front-color))
           (side-cyl0 (send self :make-marker-body pos0 side-color))
           (side-cyl1 (send self :make-marker-body pos1 side-color)))
      (send cyl1 :replace-rot (send (send thumb-link1 :copy-worldcoords) :copy-rot))
      (send side-cyl1 :replace-rot (send (send thumb-link1 :copy-worldcoords) :copy-rot))
      (send cyl2 :replace-rot (send (send thumb-link2 :copy-worldcoords) :copy-rot))
      (send side-cyl0 :replace-rot (send (send thumb-link1 :copy-worldcoords) :copy-rot))

      (send cyl0 :translate #f(2 15 0) :world)
      (send side-cyl0 :translate #f(10 26 3) :world)
      (send side-cyl0 :rotate (deg2rad 90) :x :local)
      (send cyl1 :translate #f(0 -1 -2) :local)
      (send side-cyl1 :translate #f(0 5 4) :local)
      (send side-cyl1 :rotate (deg2rad 90) :x :local)
      (send cyl2 :rotate (deg2rad -20) :y :local)
      (send cyl2 :translate #f(2 5 2) :local)
      (send thumb-link0 :assoc cyl0)
      (send thumb-link0 :assoc side-cyl0)
      (send thumb-link1 :assoc cyl1)
      (send thumb-link1 :assoc side-cyl1)
      (send thumb-link2 :assoc cyl2)
      (sethash (send thumb-link0 :name) marker-tb cyl0)
      (sethash (send thumb-link1 :name) marker-tb cyl1)
      (sethash (send thumb-link2 :name) marker-tb cyl2)
      (sethash "thumb_side_link0" marker-tb side-cyl0)
      (sethash "thumb_side_link1" marker-tb side-cyl1))

    (let* ((h-link (send self :get-link-by-name "hand_link"))
           (h-pos (send (send h-link :copy-worldcoords) :copy-pos)))
      (send self :add-palm-marker h-link (send h-link :name) h-pos front-color
            :trans #f(18 0 60))
      (send self :add-marker h-link "palm_lside_link0" h-pos side-color
            :mk-size 9 :rot-axis :z :rot-deg 104 :trans #f(0 -33.8 60))
      (send self :add-marker h-link "palm_lside_link1" h-pos side-color
            :mk-size 9 :rot-axis :z :rot-deg 104 :trans #f(0 -33.8 85))
      (send self :add-marker h-link "palm_rside_link0" h-pos side-color
            :mk-size 9 :rot-axis :z :rot-deg 76 :trans #f(5 35 74))
      (send self :add-marker h-link "palm_rside_link1" h-pos side-color
            :mk-size 9 :rot-axis :z :rot-deg 76 :trans #f(5 35 95)))
    )

  (:set-color
    (name &key rgb a)
    (let ((target (gethash name marker-tb)))
      (when target
        (let ((new-color (copy-seq (send target :get :face-color))))
          (when rgb
            (replace new-color rgb :end1 3))
          (when a
            (setf (elt new-color 3) a))
          (send target :set-color new-color)
          ))))
  )

(setq *marker* (instance marker :init *hand_v3_model*))
(send *marker* :initialize-markers)
(setq markers (send *marker* :marker-object))
(send *marker* :set-color "little_link1-r" :a 0)
(send *marker* :set-color "ring_link2-l" :a 0)
(send *marker* :set-color "ring_link1-r" :a 0)
(send *marker* :set-color "middle_link2-l" :a 0)
(send *marker* :set-color "middle_link1-r" :a 0)
(send *marker* :set-color "index_link2-l" :a 0)


(defun set-collada-body-transparency (c-body transparency)
  (unless (derivedp c-body collada-body)
    (return-from set-collada-body-transparency nil))
  (let ((glvertices (send c-body :glvertices)))
    ;; :GL-DISPLAYLIST-IDが残っていると、後から透明度を変更できない
    (gl::delete-displaylist-id (get glvertices :GL-DISPLAYLIST-ID))
    (setf (get glvertices :GL-DISPLAYLIST-ID) nil)
    (setf (get glvertices :transparent) transparency)))
;; (mapcar #'(lambda (body) (set-collada-body-transparency body 0.5)) (send *hand_v3_model* :bodies))

(unless (boundp '*irtviewer*)
  (make-irtviewer))
(send *irtviewer* :viewer :viewing :look #f(469.478 -22.6601 396.048) #f(-19.9775 11.566 370.334) #f(0.0 0.0 1.0))
(send *irtviewer* :change-background #f(0.7 0.7 0.7))
(send *irtviewer* :objects (append (list *hand_v3_model*) markers))
(send *irtviewer* :draw-objects)

(ros::roseus "visualize_marker")

(defun rank-list-desc (lst)
  (let* ((sorted-lst (sort (copy-list lst) #'>))
         (ranks (make-list (length lst) :initial-element nil)))
    (dotimes (i (length lst))
      (let ((val (nth i lst)))
        (dotimes (j (length sorted-lst))
          (when (= val (nth j sorted-lst))
            (setf (nth i ranks) (+ j 1))
            (return)))))
    ranks))
(defun sensor-to-transparency (val)
  (min 1.0 (* 2.5 (/ val 4096))))
(defun sensor-to-color (val)
  (cond
   ((= val 1) (float-vector 0.2 0.2 1.0))
   ((= val 2) (float-vector 0.2 1.0 0.2))
   ((= val 3) (float-vector 1.0 0.2 0.2))
   (t (float-vector 0.3 0.3 0.3))))

(setq sensor-order-list
      (list "little_link3" "little_link2-l" "little_link2" "little_link2-r" "little_link1-l" "little_link1"
            "ring_link3" "ring_link2" "ring_link2-r" "ring_link1-l" "ring_link1"
            "middle_link3" "middle_link2" "middle_link2-r" "middle_link1-l" "middle_link1"
            "index_link3" "index_link2" "index_link2-r" "index_link1-l" "index_link1" "index_link1-r"
            "thumb_link2" "thumb_link1" "thumb_side_link1" "thumb_link0" "thumb_side_link0"
            "palm_lside_link1" "palm_lside_link0" "palm_rside_link1" "palm_rside_link0" "hand_link"))

(defun sensor-cb (msg)
  (let* ((raw-data (coerce (send msg :data) cons))
         (ranks (rank-list-desc raw-data)))
    (mapc #'(lambda (name val rank)
              (send *marker* :set-color name
                    :rgb (sensor-to-color rank)        ;; ランクから色を決定
                    :a   (sensor-to-transparency val)) ;; 値から透明度を決定
              )
          sensor-order-list
          raw-data
          ranks))
  (send *irtviewer* :draw-objects))

(ros::subscribe "force_data_diff" std_msgs::Float32MultiArray #'sensor-cb)

(do-until-key
 (ros::spin-once)
 (x::window-main-one)
 )
