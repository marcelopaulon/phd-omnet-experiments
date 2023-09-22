package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_8
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=10.91,7867.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=746.45,7822.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1239.87,7810.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1904.62,7851.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3842.89,7858.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4501.10,7854.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4893.92,7761.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=336.08,7574.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1534.73,7590.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2276.69,7490.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2966.74,7607.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3996.56,7503.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3628.56,7369.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4599.71,7306.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=939.22,7185.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1400.01,7059.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2658.92,7016.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3108.43,7108.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5186.01,7049.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5592.61,6992.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=416.77,6872.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=896.37,6812.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1893.14,6843.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3736.15,6867.73");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=623.60,6564.95");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1198.15,6602.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1686.74,6525.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2404.53,6469.78");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2897.49,6478.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3301.56,6531.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3946.93,6500.86");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4507.38,6643.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5189.81,6677.39");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=220.02,6302.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5885.38,6388.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=650.96,5965.64");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1173.77,6078.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1691.85,5974.37");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3126.75,6177.72");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4459.67,6039.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5220.45,6079.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5727.43,5960.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=128.94,5826.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1992.74,5761.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2419.42,5832.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3530.78,5773.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=523.92,5487.21");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=877.98,5639.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1422.39,5607.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2428.94,5418.47");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2896.48,5644.71");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3926.68,5669.31");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4280.40,5542.41");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5041.60,5539.61");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5663.52,5488.33");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=893.45,5231.63");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1994.83,5349.92");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3850.54,5279.07");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4710.52,5288.47");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=512.98,5071.29");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1433.20,4935.78");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2583.36,5026.27");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2997.73,5124.68");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3486.61,5074.05");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5273.12,5102.79");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=813.56,4817.77");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1785.89,4758.04");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2748.53,4682.01");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4122.76,4843.86");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5155.19,4754.35");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5846.28,4833.19");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1084.46,4510.97");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1538.99,4454.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2277.06,4586.74");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3461.25,4620.11");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3852.69,4384.46");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4710.24,4615.63");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=654.17,4193.20");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1796.05,4174.11");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2193.42,4226.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2674.85,4319.33");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3266.59,4179.85");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4404.09,4266.27");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4953.87,4125.18");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5467.04,4269.69");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=318.06,3964.71");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=996.28,3996.19");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1584.78,3860.05");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3613.50,3997.87");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5484.70,3885.78");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5901.46,4045.49");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1063.99,3605.10");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2630.34,3833.99");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3446.17,3665.83");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4056.11,3723.67");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4988.42,3652.96");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=583.91,3542.78");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2160.63,3545.70");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2892.13,3567.65");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4243.73,3387.91");
    }

}

