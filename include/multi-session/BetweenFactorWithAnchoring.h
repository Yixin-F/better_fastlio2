#pragma once

#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class BetweenFactorWithAnchoring: public NoiseModelFactor4<VALUE, VALUE, VALUE, VALUE> {

    // Check that VALUE type is a testable Lie group
    BOOST_CONCEPT_ASSERT((IsTestable<VALUE>));
    BOOST_CONCEPT_ASSERT((IsLieGroup<VALUE>));

  public:

    typedef VALUE T;

  private:

    typedef BetweenFactorWithAnchoring<VALUE> This;
    typedef NoiseModelFactor4<VALUE, VALUE, VALUE, VALUE> Base;

    VALUE measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<BetweenFactorWithAnchoring> shared_ptr;

    /** default constructor - only use for serialization */
    BetweenFactorWithAnchoring() {}

    /** Constructor */
    BetweenFactorWithAnchoring(
          // 1 for robot 1, and 2 for robot 2
          Key key1, Key key2, Key anchor_key1, Key anchor_key2,
          const VALUE& measured,
          const SharedNoiseModel& model = nullptr) :
      Base(model, key1, key2, anchor_key1, anchor_key2), measured_(measured) {
    }

    virtual ~BetweenFactorWithAnchoring() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BetweenFactorWithAnchoring("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      traits<T>::Print(measured_, "  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<T>::Equals(this->measured_, e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */

    // some useful link, giseop 
    // line 384, https://gtsam.org/doxygen/a00317_source.html
    // https://gtsam.org/doxygen/a02091.html
    // betweenfactor https://gtsam.org/doxygen/a00935_source.html
    // line 224 https://gtsam.org/doxygen/a00053_source.html
    // isam ver. line 233, https://people.csail.mit.edu/kaess/isam/doc/slam2d_8h_source.html
    /** vector of errors */
    Vector evaluateError(
        const T& p1, const T& p2, const T& anchor_p1, const T& anchor_p2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> anchor_H1 = boost::none,
        boost::optional<Matrix&> anchor_H2 = boost::none
      ) const {

      // anchor node h(.) (ref: isam ver. line 233, https://people.csail.mit.edu/kaess/isam/doc/slam2d_8h_source.html)
      T hx1 = traits<T>::Compose(anchor_p1, p1, anchor_H1, H1); // for the updated jacobian, see line 60, 219, https://gtsam.org/doxygen/a00053_source.html
      T hx2 = traits<T>::Compose(anchor_p2, p2, anchor_H2, H2); 
      T hx = traits<T>::Between(hx1, hx2, H1, H2); 

      return traits<T>::Local(measured_, hx);
    }

    /** return the measured */
    const VALUE& measured() const {
      return measured_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 4;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor4",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }

	//   // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
	//   enum { NeedsToAlign = (sizeof(VALUE) % 16) == 0 };
  //   public:
  //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  }; // \class BetweenFactorWithAnchoring


  // traits
  template<class VALUE>
  struct traits<BetweenFactorWithAnchoring<VALUE> > : public Testable<BetweenFactorWithAnchoring<VALUE> > {};

  // /**
  //  * Binary between constraint - forces between to a given value
  //  * This constraint requires the underlying type to a Lie type
  //  *
  //  */
  // template<class VALUE>
  // class BetweenConstraintGiseop : public BetweenFactorWithAnchoring<VALUE> {
  // public:
  //   typedef boost::shared_ptr<BetweenConstraintGiseop<VALUE> > shared_ptr;

  //   /** Syntactic sugar for constrained version */
  //   BetweenConstraintGiseop(const VALUE& measured, Key key1, Key key2, double mu = 1000.0) :
  //     BetweenFactorWithAnchoring<VALUE>(key1, key2, measured,
  //                          noiseModel::Constrained::All(traits<VALUE>::GetDimension(measured), std::abs(mu)))
  //   {}

  // private:

  //   /** Serialization function */
  //   friend class boost::serialization::access;
  //   template<class ARCHIVE>
  //   void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
  //     ar & boost::serialization::make_nvp("BetweenFactorWithAnchoring",
  //         boost::serialization::base_object<BetweenFactorWithAnchoring<VALUE> >(*this));
  //   }
  // }; // \class BetweenConstraintGiseop

  // /// traits
  // template<class VALUE>
  // struct traits<BetweenConstraintGiseop<VALUE> > : public Testable<BetweenConstraintGiseop<VALUE> > {};

} /// namespace gtsam