#pragma once

#include <cstdint>

namespace R1video
{
    namespace Config
    {
        inline constexpr double body_radius{/*TODO*/};
        inline constexpr double wheel_radius{/*TODO*/};

        namespace Limitation
        {
            // 0に設定すると制限がかからなくなる。
            inline constexpr double wheel_vela{/*TODO*/10};
            inline constexpr double wheel_acca{/*TODO*/10};
        
            inline constexpr double body_vell_ratio{/*TODO*/0.9};
            inline constexpr double body_vela_ratio{/*TODO*/0.1};

            inline constexpr double body_vell{/*TODO*/wheel_vela * body_vell_ratio};
            inline constexpr double body_vela{/*TODO*/wheel_vela * body_vela_ratio};
        }


        namespace ExecutionInterval
        {
            inline constexpr double under_carriage_freq{/*TODO*/1000};
            inline constexpr double manual_commander_freq{/*TODO*/1000};
            inline constexpr double auto_commander_freq{/*TODO*/1000};
            inline constexpr double not_emergency_signal_freq{/*TODO*/100};
        }

        namespace CanId
        {
            namespace Tx
            {
                namespace DriveMotor
                {
                    inline constexpr std::uint16_t R{/*TODO*/};
                    inline constexpr std::uint16_t L{/*TODO*/};
                    inline constexpr std::uint16_t all[2]{R ,L};
                }

                namespace LiftMotor
                {
                    inline constexpr std::uint16_t R{/*TODO*/};
                    inline constexpr std::uint16_t L{/*TODO*/};

                    inline constexpr std::uint16_t subX{/*TODO*/};
                    inline constexpr std::uint16_t subY{/*TODO*/};

                    inline constexpr std::uint16_t collect{/*TODO*/};
                }

                inline constexpr std::uint16_t position_controll_ids[] = {LiftMotor::R, LiftMotor::L, LiftMotor::subX, LiftMotor::subY, LiftMotor::collect};
                inline constexpr std::size_t position_controll_ids_size = sizeof(position_controll_ids) / sizeof(position_controll_ids[0]);

                namespace Emergency
                {
                    inline constexpr std::uint16_t power{/*TODO*/0x0};
                }

                // まだ数枚ある
            }

            namespace Rx
            {
                inline constexpr std::uint16_t odometry{/*TODO*/0x10};
                inline constexpr std::uint16_t stopped{/*TODO*/0x8};
            }
        }
    }
}
