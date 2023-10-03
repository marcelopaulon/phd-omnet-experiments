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

network DADCA_ACK_n100_u3_a8000m2_6
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
        @display("i=misc/sensor;p=1048.74,7923.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3733.46,7969.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4253.73,7786.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5001.01,7879.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5397.79,7939.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5954.07,7909.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=272.50,7731.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=696.26,7734.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1576.46,7737.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2198.00,7581.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2878.55,7673.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3988.30,7527.80");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4581.32,7641.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5336.65,7548.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=991.52,7385.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1357.44,7453.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3459.62,7301.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4710.77,7292.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5068.46,7293.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5663.14,7391.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=395.30,7090.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=888.97,7042.49");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1622.57,7083.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2914.05,7256.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3974.92,7125.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4467.86,7011.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5405.78,7137.81");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2278.01,6987.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5626.60,6824.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=27.11,6520.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1414.17,6737.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1901.22,6611.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2658.08,6575.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3221.20,6626.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3814.11,6702.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4421.27,6655.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4832.40,6703.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5202.81,6651.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=600.72,6499.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4033.84,6329.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4453.39,6285.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5884.88,6485.11");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=911.03,6087.00");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1453.06,6137.50");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1986.98,6240.29");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2418.57,6191.70");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3126.47,6146.45");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4774.36,6133.81");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5288.47,6165.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5648.34,6126.52");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=28.18,5833.13");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=390.71,5863.97");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4156.85,5975.12");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=745.62,5633.67");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1707.68,5727.56");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2180.40,5744.06");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2757.38,5768.01");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3126.54,5573.18");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3511.52,5745.05");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3862.48,5615.87");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4397.63,5675.07");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5240.17,5740.06");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5735.79,5571.79");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=387.46,5376.93");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1269.86,5510.00");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2228.83,5368.18");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4819.96,5513.84");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=943.73,5198.69");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1628.65,5092.67");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2745.30,5131.62");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3150.93,5060.81");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3799.74,5223.31");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4786.57,5126.67");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5404.24,5157.23");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1132.64,4885.61");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1949.87,4893.54");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4133.78,4895.96");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5843.14,4880.17");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=449.26,4763.71");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2424.35,4682.65");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3647.55,4655.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4672.55,4607.83");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5322.47,4711.03");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=129.43,4500.65");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=826.48,4522.29");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1282.71,4484.91");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2208.75,4387.82");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2599.35,4332.20");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3099.94,4530.18");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4153.81,4488.54");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5046.90,4294.40");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=544.30,4087.23");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3003.18,4192.45");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3477.61,4280.81");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4431.63,4107.42");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5733.17,4218.00");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=99.75,3887.63");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1233.43,3917.06");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1597.18,3883.73");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2074.98,4035.76");
    }

}

