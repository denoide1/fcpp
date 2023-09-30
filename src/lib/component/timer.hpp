// Copyright © 2023 Giorgio Audrito. All Rights Reserved.

/**
 * @file timer.hpp
 * @brief Implementation of the `timer` component managing round executions.
 */

#ifndef FCPP_COMPONENT_TIMER_H_
#define FCPP_COMPONENT_TIMER_H_

#include <cassert>
#include <type_traits>

#include "lib/component/base.hpp"
#include "lib/data/field.hpp"


/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {


// Namespace for all FCPP components.
namespace component {


// Namespace of tags to be used for initialising components.
namespace tags {
    //! @brief Node initialisation tag associating to a starting time of execution (defaults to \ref TIME_MAX).
    struct start {};

    //! @brief Net initialisation tag associating to a factor to be applied to real time (defaults to \ref FCPP_REALTIME if not infinite).
    struct realtime_factor {};
}


/**
 * @brief Component managing round executions.
 *
 * It warps the times of events generated by ancestor components (and not by descendent components).
 * The \ref timer component cannot be a parent of a \ref scheduler otherwise round planning may not work.
 * A \ref simulated_connector component cannot be a parent of a \ref timer otherwise round planning may block message exchange.
 *
 * <b>Declaration flags:</b>
 * - \ref tags::realtime defines whether running should follow real time (defaults to `FCPP_REALTIME < INF`).
 *
 * <b>Node initialisation tags:</b>
 * - \ref tags::start associates to a starting time of execution (defaults to \ref TIME_MAX).
 *
 * <b>Net initialisation tags:</b>
 * - \ref tags::realtime_factor associates to a `real_t` factor to be applied to real time (defaults to \ref FCPP_REALTIME if not infinite).
 */
template <class... Ts>
struct timer {
    //! @brief Whether running should follow real time.
    constexpr static bool realtime = common::option_flag<tags::realtime, FCPP_REALTIME < INF, Ts...>;

    /**
     * @brief The actual component.
     *
     * Component functionalities are added to those of the parent by inheritance at multiple levels: the whole component class inherits tag for static checks of correct composition, while `node` and `net` sub-classes inherit actual behaviour.
     * Further parametrisation with F enables <a href="https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern">CRTP</a> for static emulation of virtual calls.
     *
     * @param F The final composition of all components.
     * @param P The parent component to inherit from.
     */
    template <typename F, typename P>
    struct component : public P {
        //! @cond INTERNAL
        DECLARE_COMPONENT(timer);
        AVOID_COMPONENT(timer,connector);
        CHECK_COMPONENT(calculus);
        //! @endcond

        //! @brief The local part of the component.
        class node : public P::node {
          public: // visible by net objects and the main program
            /**
             * @brief Main constructor.
             *
             * @param n The corresponding net object.
             * @param t A `tagged_tuple` gathering initialisation values.
             */
            template <typename S, typename T>
            node(typename F::net& n, common::tagged_tuple<S,T> const& t) : P::node(n,t), m_neigh(TIME_MIN) {
                m_prev = m_cur = TIME_MIN;
                m_next = common::get_or<tags::start>(t, TIME_MAX);
                m_offs = (m_next == TIME_MAX ? 0 : m_next);
                m_fact = 1;
            }

            /**
             * @brief Returns next event to schedule for the node component.
             *
             * Should correspond to the next time also during updates.
             */
            times_t next() const {
                return m_next < TIME_MAX ? m_next : m_offs < TIME_MAX and P::node::next() < TIME_MAX ? P::node::next()/m_fact + m_offs : TIME_MAX;
            }

            //! @brief Updates the internal status of node component.
            void update() {
                m_prev = m_cur;
                m_cur = next();
                fcpp::details::self(m_neigh, P::node::uid) = m_prev;
                if (m_next < TIME_MAX) {
                    // next round was planned
                    m_next = TIME_MAX;
                    P::node::round(m_cur);
                } else {
                    // next round was scheduled
                    P::node::update();
                }
            }

            //! @brief Performs computations at round start with current time `t`.
            void round_start(times_t t) {
                P::node::round_start(t);
                maybe_align_inplace(m_neigh, has_calculus<P>{});
            }

            //! @brief Receives an incoming message (possibly reading values from sensors).
            template <typename S, typename T>
            void receive(times_t t, device_t d, common::tagged_tuple<S,T> const& m) {
                P::node::receive(t, d, m);
                fcpp::details::self(m_neigh, d) = t;
            }

            //! @brief Returns the time of the second most recent round (previous during rounds).
            times_t previous_time() const {
                return m_prev;
            }

            //! @brief Returns the time of the most recent round (current during rounds).
            times_t current_time() const {
                return m_cur;
            }

            //! @brief Returns the time of the next scheduled round.
            times_t next_time() const {
                return next();
            }

            //! @brief Plans the time of the next round (`TIME_MAX` to indicate termination).
            void next_time(times_t t) {
                if (t < TIME_MAX) {
                    assert(m_offs < TIME_MAX);
                    m_offs += t - (m_next < TIME_MAX ? m_next : m_cur);
                } else m_offs = TIME_MAX;
                m_next = t;
            }

            //! @brief Terminate round executions.
            void terminate() {
                m_next = m_offs = TIME_MAX;
            }

            //! @brief Returns the time stamps of the most recent messages from neighbours.
            field<times_t> const& message_time() const {
                return m_neigh;
            }

            //! @brief Returns the time difference with neighbours.
            field<times_t> nbr_lag() const {
                return m_cur - m_neigh;
            }

            //! @brief Returns the warping factor applied to following schedulers.
            real_t frequency() const {
                return m_fact;
            }

            //! @brief Sets the warping factor applied to following schedulers.
            void frequency(real_t f) {
                if (m_offs < TIME_MAX) m_offs = m_cur - times_t(m_fact*(m_cur - m_offs)/f);
                m_fact = f;
            }

          private: // implementation details
            //! @brief Changes the domain of a field-like structure to match the domain of the neightbours ids.
            template <typename A>
            void maybe_align_inplace(field<A>& x, std::true_type) {
                align_inplace(x, std::vector<device_t>(fcpp::details::get_ids(P::node::nbr_uid())));
            }

            //! @brief Does not perform any alignment
            template <typename A>
            void maybe_align_inplace(field<A>&, std::false_type) {}

            //! @brief Times of previous, current and next planned rounds.
            times_t m_prev, m_cur, m_next;

            //! @brief Times of neighbours.
            field<times_t> m_neigh;

            //! @brief Offset between the following schedule and actual times.
            times_t m_offs;

            //! @brief Warping factor for the following schedule.
            real_t m_fact;
        };

        //! @brief The global part of the component.
        class net : public P::net {
            static_assert(FCPP_REALTIME >= 0, "time cannot flow backwards");

          public: // visible by node objects and the main program
            //! @brief Constructor from a tagged tuple.
            template <typename S, typename T>
            explicit net(common::tagged_tuple<S,T> const& t) : P::net(t) {
                m_offs = 0;
                m_fact = common::get_or<tags::realtime_factor>(t, FCPP_REALTIME < INF ? FCPP_REALTIME : 1);
                m_inv = 1/m_fact;
                m_last_update = m_next_update = 0;
                assert(m_fact >= 0);
            }

            /**
             * @brief Returns next event to schedule for the net component.
             *
             * Should correspond to the next time also during updates.
             */
            times_t next() const {
                m_next_update = P::net::next();
                return m_offs < TIME_MAX and m_fact > 0 and m_next_update < TIME_MAX ? m_next_update * m_inv + m_offs : TIME_MAX;
            }

            //! @brief Updates the internal status of net component.
            void update() {
                m_last_update = m_next_update;
                P::net::update();
            }

            //! @brief A measure of the internal time clock.
            times_t internal_time() const {
                if (not realtime or m_last_update == m_next_update or m_fact == 0 or m_inv == 0 or m_offs == TIME_MAX) return m_last_update;
                times_t t = (P::net::real_time() - m_offs) * m_fact;
                return std::max(std::min(t, m_next_update), m_last_update);
            }

            //! @brief Terminate round executions.
            void terminate() {
                m_offs = TIME_MAX;
            }

            //! @brief Returns the warping factor applied to following schedulers.
            real_t frequency() const {
                return m_fact;
            }

            //! @brief Sets the warping factor applied to following schedulers.
            void frequency(real_t f) {
                assert(f >= 0);
                if (m_offs == TIME_MAX) return; // execution terminated, nothing to do
                if (f > 0 and m_fact > 0)
                    m_offs += m_last_update * (m_inv - 1/f);
                else if (f > 0) // resume
                    m_offs = P::net::as_final().next() - m_next_update / f;
                m_fact = f;
                m_inv = 1/f;
            }

          private: // implementation details
            //! @brief Offset between the following schedule and actual times.
            times_t m_offs;

            //! @brief Warping factor and inverse for the following schedule.
            real_t m_fact, m_inv;

            //! @brief The internal time of the last update.
            times_t m_last_update;

            //! @brief The internal time of the next update.
            mutable times_t m_next_update;
        };
    };
};


}


}

#endif // FCPP_COMPONENT_TIMER_H_
