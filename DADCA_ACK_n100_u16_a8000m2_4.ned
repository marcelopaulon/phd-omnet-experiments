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

network DADCA_ACK_n100_u16_a8000m2_4
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
        @display("i=misc/sensor;p=219.76,7909.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1582.85,7779.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1927.57,7874.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2582.08,7948.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3169.56,7855.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3820.64,7780.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5454.67,7833.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5875.15,7932.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=702.95,7679.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1080.82,7552.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4413.05,7684.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4926.87,7661.79");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5895.60,7570.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=480.19,7286.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2058.67,7458.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2433.04,7375.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3081.14,7287.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3679.20,7409.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4091.99,7353.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5543.67,7485.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1005.52,7087.92");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1451.49,7189.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1803.74,7128.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2788.85,7054.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3822.39,7083.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4439.76,7187.97");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4889.40,7182.41");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5494.12,7091.89");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5885.59,7125.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=279.85,6895.59");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1905.17,6763.27");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3437.99,6825.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4146.02,6796.19");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4653.20,6769.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5189.06,6766.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=745.88,6672.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1477.51,6725.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2335.04,6630.99");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2980.96,6649.87");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3676.05,6519.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1062.30,6417.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1668.88,6406.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3397.49,6293.94");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3994.94,6351.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4387.92,6444.08");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5217.59,6413.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5695.50,6425.03");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=501.42,6182.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2222.87,6193.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2776.90,6131.00");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3684.55,6024.83");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4846.84,6161.42");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5347.77,6032.14");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1238.37,5941.21");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4509.30,6001.70");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5098.86,5786.02");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5736.20,5957.02");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=305.85,5688.73");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1812.10,5668.76");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2466.88,5741.61");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3115.09,5686.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3478.67,5640.13");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4688.23,5601.10");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=426.58,5288.94");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1435.36,5294.90");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3930.63,5392.03");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4421.44,5322.30");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5522.26,5322.44");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=10.31,5188.59");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=940.04,5132.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1757.01,5132.02");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2551.69,5221.37");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3236.79,5263.83");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5038.80,5181.00");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=279.42,4841.81");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1262.19,4814.78");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2047.20,4880.00");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2544.98,4828.37");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2927.84,4999.10");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3877.12,4898.02");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4682.83,4909.36");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5237.00,4880.16");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5626.75,4840.94");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1779.38,4543.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3220.06,4763.75");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3570.62,4679.44");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4371.26,4720.04");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4972.17,4617.93");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=546.05,4486.41");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=910.04,4333.41");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2294.37,4402.29");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2660.93,4484.08");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3017.07,4460.29");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5309.77,4452.06");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=99.49,4059.73");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=582.15,4049.13");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1387.08,4130.72");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2034.99,4045.35");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3399.41,4070.35");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3792.60,4045.84");
    }

}

