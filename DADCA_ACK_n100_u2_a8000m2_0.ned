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

network DADCA_ACK_n100_u2_a8000m2_0
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
        @display("i=misc/sensor;p=968.71,7878.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1762.29,7986.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2902.65,7807.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5054.51,7790.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=263.63,7673.54");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1330.30,7676.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2082.84,7598.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2445.88,7603.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3368.09,7605.44");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3899.05,7739.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4488.82,7536.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5754.38,7549.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5.84,7386.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=876.46,7475.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1764.48,7291.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2942.16,7359.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3967.47,7366.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5076.05,7378.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=523.36,7234.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1260.73,7061.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2134.77,7143.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3482.19,7120.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4164.72,7016.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4765.13,7165.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=306.92,6769.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2555.96,6861.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4463.24,6832.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4885.82,6829.01");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5325.84,6957.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5759.00,6998.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=664.95,6702.55");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1685.61,6726.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2104.74,6736.09");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3262.89,6754.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3860.25,6687.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1410.26,6439.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2573.78,6316.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3240.99,6382.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4237.66,6484.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4758.96,6372.79");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5185.31,6505.67");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5521.65,6388.14");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5969.35,6450.56");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=46.84,6174.58");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=614.56,6117.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=974.86,6158.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2003.40,6220.28");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3082.86,6038.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4018.38,6144.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5631.24,6042.88");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1555.45,5875.14");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2326.76,5814.46");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2704.88,5902.29");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3572.64,5856.43");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4411.71,6004.02");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5278.63,5978.50");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=58.72,5740.20");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=455.13,5694.52");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=910.16,5612.02");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3275.01,5663.39");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3989.72,5629.69");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5114.35,5623.55");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5744.12,5533.64");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2032.29,5438.40");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2851.21,5397.78");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4676.20,5317.35");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5394.29,5322.55");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=21.23,5101.61");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=749.15,5275.33");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1487.87,5196.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2281.80,5079.66");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3731.28,5165.32");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4404.34,5095.76");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5966.89,5112.67");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=372.00,4940.79");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1003.43,4920.11");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1742.31,4789.31");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2644.40,4947.71");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5056.73,4990.22");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5696.69,4822.63");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2226.38,4702.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3157.86,4773.12");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3565.34,4633.35");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4107.03,4681.17");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5452.78,4556.03");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5997.43,4562.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=188.24,4336.93");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=643.95,4453.73");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1791.84,4391.22");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2494.14,4302.53");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3039.68,4353.88");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4047.54,4317.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4637.54,4440.08");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1262.92,4270.03");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4566.83,4044.26");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5007.49,4092.63");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5597.00,4228.39");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=255.75,3799.13");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=557.47,3990.39");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=947.22,3831.97");
    }

}

