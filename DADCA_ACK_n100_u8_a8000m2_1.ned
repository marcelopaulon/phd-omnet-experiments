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

network DADCA_ACK_n100_u8_a8000m2_1
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
        @display("i=misc/sensor;p=74.68,7825.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1896.67,7957.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2402.70,7829.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3082.16,7929.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3520.25,7988.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3976.75,7758.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4939.96,7806.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5628.84,7909.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=417.82,7657.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=892.32,7531.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1500.04,7620.27");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3382.32,7638.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4389.34,7661.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2072.62,7416.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2461.96,7278.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2782.91,7468.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3836.34,7333.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5113.12,7479.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5540.14,7413.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=34.38,7139.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=600.87,7229.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=943.38,7022.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1412.24,7215.60");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2060.04,7040.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3359.71,7155.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4073.30,7038.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4840.31,7216.97");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5226.40,7066.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5873.28,7248.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=393.18,6866.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1681.62,6904.19");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2857.10,6874.36");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3285.48,6796.95");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4599.23,6881.86");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=120.97,6571.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2415.21,6658.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4938.18,6573.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5583.95,6692.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=565.04,6470.46");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1184.45,6456.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1640.50,6454.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2039.20,6487.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2989.96,6334.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3494.41,6495.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3856.43,6384.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4465.65,6496.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=256.06,6247.29");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=880.27,6083.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1791.78,6108.53");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2345.88,6255.88");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3496.34,6085.89");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4173.79,6103.57");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5145.11,6129.05");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5827.67,6245.83");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=10.41,5861.06");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1373.17,5962.28");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2247.71,5835.40");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2672.33,5966.29");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3093.98,5877.10");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4530.22,6003.26");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5961.75,5788.05");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=564.79,5703.48");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1887.59,5604.26");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3720.96,5729.38");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4595.34,5542.28");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5123.06,5723.44");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=400.17,5320.73");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1156.52,5430.11");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2202.28,5368.93");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2905.13,5507.42");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3340.05,5277.96");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3773.74,5328.15");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4203.48,5523.94");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5307.17,5296.08");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=65.76,5170.38");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=798.52,5045.39");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2638.29,5180.16");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4403.99,5148.84");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5777.45,5245.35");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1598.06,4927.55");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3036.92,4949.17");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3850.82,4893.54");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4215.90,4850.55");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4972.17,4980.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5543.05,4943.23");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=161.48,4690.47");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=581.82,4649.73");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3317.41,4678.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4709.98,4543.72");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=802.51,4321.62");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1202.50,4405.03");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2013.30,4321.03");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2355.90,4509.36");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2936.71,4362.33");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4048.96,4375.74");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5276.68,4493.54");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5619.04,4369.51");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1616.01,4150.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3551.36,4123.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4711.16,4112.61");
    }

}

