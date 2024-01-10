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

network DADCA_ACK_n100_u4_a8000m2_0
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
        @display("i=misc/sensor;p=460.96,7757.31");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1216.11,7982.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1699.89,7799.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2090.09,7781.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2524.95,7969.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3011.36,7941.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3732.67,7919.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4237.38,7916.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4621.13,7925.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1033.46,7672.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3557.09,7599.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4099.74,7531.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5898.77,7573.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=35.89,7487.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=704.80,7395.06");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1814.23,7289.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2333.01,7339.94");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2957.66,7440.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4572.37,7433.25");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5028.13,7452.50");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5540.79,7337.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1291.03,7205.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3252.28,7099.60");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4447.58,7087.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4800.65,7110.13");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5312.51,7032.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=311.61,6892.76");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=858.75,6853.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1494.30,6892.98");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2633.83,6891.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3056.07,6785.96");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3734.84,6949.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5881.76,7001.94");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1230.76,6650.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2174.69,6711.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4049.34,6757.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4613.82,6559.38");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5206.68,6572.23");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5554.52,6663.41");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=200.21,6497.11");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=793.54,6401.99");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1123.00,6278.89");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1476.27,6376.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2608.03,6344.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3627.04,6410.30");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5911.21,6285.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=419.47,6137.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2044.23,6064.86");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3100.41,6184.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4113.12,6108.37");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4503.28,6204.68");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=707.64,5877.73");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1371.75,5822.19");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1729.74,5886.35");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2848.28,5877.90");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3606.98,5923.42");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4433.04,5833.75");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5008.10,5950.32");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5617.61,5980.24");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=35.31,5540.03");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2306.23,5644.72");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3284.93,5670.32");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3788.58,5598.78");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4980.89,5538.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5799.73,5617.27");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=936.44,5487.34");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1963.12,5366.76");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3130.63,5309.63");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4149.31,5361.18");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5436.80,5441.73");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=420.25,5147.92");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=779.33,5144.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1497.62,5276.86");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3568.05,5088.27");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4684.98,5232.67");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5180.37,5133.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1602.00,4799.42");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2070.56,4999.66");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2824.68,4791.29");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3248.58,4886.17");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4140.09,4807.57");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4868.49,4925.88");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=100.33,4705.81");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=819.97,4634.33");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1197.06,4748.54");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5362.39,4637.48");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=380.25,4355.28");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1980.56,4353.11");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2523.60,4369.53");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3222.89,4500.38");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3861.20,4443.57");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4528.24,4505.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=667.91,4060.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1244.32,4264.55");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1685.77,4078.75");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2983.07,4105.50");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4931.73,4100.30");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5302.26,4221.64");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=27.73,4033.32");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3414.03,3953.14");
    }

}

