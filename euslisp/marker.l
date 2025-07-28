(load "../urdf/hand_v3.l")

(hand_v3_model)

;; ("base_link" "adduction_pulley_link" "arm_link1" "arm_link2" "arm_link3" "hand_link" "index_link0" "index_link1" "index_link2" "index_link3" "index_pulley_link" "little_link0" "little_link1" "little_link2" "little_link3" "little_pulley_link" "middle_link0" "middle_link1" "middle_link2" "middle_link3" "middle_pulley_link" "ring_link0" "ring_link1" "ring_link2" "ring_link3" "ring_pulley_link" "thumb_link0" "thumb_link1" "thumb_link2" "thumb_pulley_link")


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
    (let ((value-list nil))
      (maphash #'(lambda (key val) (push val value-list))
               marker-tb)
      (nreverse (flatten value-list))))

  (:make-marker
    (pos color)
    (setq cyl (make-cylinder 6 1))
    (send cyl :replace-pos pos)
    (send cyl :rotate (/ pi 2) :y)
    (send cyl :translate (float-vector 3 0 0) :world)
    (send cyl :set-color color)
    cyl)

  (:initialize-markers
    ()
    (setq marker-tb (make-hash-table :test #'string-equal))
    (dolist (finger (list "index" "middle" "ring" "little"))
      (let* ((link1 (send self :get-link-by-name (format nil "~A_link1" finger)))
             (link2 (send self :get-link-by-name (format nil "~A_link2" finger)))
             (link3 (send self :get-link-by-name (format nil "~A_link3" finger)))
             (pos1 (scale 0.5 (v+ (send (send link1 :copy-worldcoords) :pos) (send (send link2 :copy-worldcoords) :pos))))
             (pos2 (scale 0.5 (v+ (send (send link2 :copy-worldcoords) :pos) (send (send link3 :copy-worldcoords) :pos))))
             (pos3 (send (send (send link3 :copy-worldcoords) :translate (float-vector -1.5 0 12) :world) :copy-pos))
             (cyl1 (send self :make-marker pos1 #f(0.65 0.65 1.0 0.6)))
             (cyl1-l (send self :make-marker pos1 #f(0.65 0.65 1.0 0.6)))
             (cyl1-r (send self :make-marker pos1 #f(0.65 0.65 1.0 0.6)))
             (cyl2 (send self :make-marker pos2 #f(0.65 0.65 1.0 0.6)))
             (cyl2-l (send self :make-marker pos2 #f(0.65 0.65 1.0 0.6)))
             (cyl2-r (send self :make-marker pos2 #f(0.65 0.65 1.0 0.6)))
             (cyl3 (send self :make-marker pos3 #f(0.65 0.65 1.0 0.6))))
        (send cyl1-l :rotate (deg2rad -80) :z :world)
        (send cyl1-l :translate (float-vector -5 -6 0) :world)
        (send cyl1-r :rotate (deg2rad 80) :z :world)
        (send cyl1-r :translate (float-vector -5 6 0) :world)
        (send cyl2-l :rotate (deg2rad -80) :z :world)
        (send cyl2-l :translate (float-vector -5 -6 0) :world)
        (send cyl2-r :rotate (deg2rad 80) :z :world)
        (send cyl2-r :translate (float-vector -5 6 0) :world)
        (send cyl3 :rotate (deg2rad -12) :y)
        (send link1 :assoc cyl1)
        (send link1 :assoc cyl1-l)
        (send link1 :assoc cyl1-r)
        (send link2 :assoc cyl2)
        (send link1 :assoc cyl2-l)
        (send link1 :assoc cyl2-r)
        (send link3 :assoc cyl3)
        (sethash (send link1 :name) marker-tb (list cyl1 cyl1-l cyl1-r))
        (sethash (send link2 :name) marker-tb (list cyl2 cyl2-l cyl2-r))
        (sethash (send link3 :name) marker-tb cyl3)
        )))

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
