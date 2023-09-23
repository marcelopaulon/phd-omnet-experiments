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

network DADCA_ACK_n100_u2_a8000m2_6
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
        @display("i=misc/sensor;p=979.32,7848.31");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1531.07,7914.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1946.52,7833.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2473.29,7848.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3024.84,7964.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3424.27,7957.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3840.92,7991.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4497.31,7871.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5535.28,7869.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=332.18,7745.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2682.02,7549.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3123.64,7546.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3535.46,7599.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5092.01,7719.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5909.59,7537.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=161.90,7377.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=598.17,7390.13");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=982.81,7258.79");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1369.35,7469.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2047.80,7332.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4347.17,7288.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4790.85,7449.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5189.36,7335.25");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5535.10,7477.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=408.01,7053.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2799.25,7065.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3437.42,7184.54");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3975.94,7044.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5598.65,7059.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=857.67,6912.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1387.47,6853.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2197.83,6890.26");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3138.58,6887.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4751.57,6857.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5905.31,6873.97");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=175.93,6699.81");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=586.02,6546.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1806.97,6652.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2682.16,6674.41");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3820.68,6622.02");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4183.62,6750.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5286.90,6636.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1207.95,6300.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2373.41,6488.73");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2960.93,6380.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3487.54,6391.78");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5712.08,6467.19");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4126.59,6213.32");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4689.12,6171.26");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5889.21,6034.24");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=553.74,6007.48");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=996.86,5879.81");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1481.39,5914.68");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2391.47,5974.14");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3016.75,5799.77");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3557.84,5990.31");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5204.82,6012.01");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5570.50,5885.01");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=34.64,5770.29");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2090.00,5727.36");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4508.11,5684.26");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5948.74,5627.31");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=144.14,5315.34");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=609.85,5357.98");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1608.32,5335.99");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2719.34,5493.96");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3648.92,5408.59");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4120.14,5494.77");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5046.21,5363.75");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5448.12,5516.74");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5774.25,5320.31");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1079.82,5265.54");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1963.22,5103.13");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2518.00,5095.76");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3290.30,5160.14");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3926.61,5176.96");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4703.02,5235.48");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5343.07,5087.55");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=817.24,4869.80");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1345.85,4802.90");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4374.65,5024.11");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=245.33,4674.33");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1935.08,4746.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2854.46,4773.80");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3209.93,4611.93");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3567.09,4760.18");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4363.78,4625.82");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4949.34,4690.60");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5738.63,4737.38");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1082.30,4509.61");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1767.50,4313.71");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2255.57,4467.66");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2616.88,4403.90");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3838.58,4521.71");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5382.62,4480.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=241.16,4201.19");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=623.55,4207.82");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=954.09,4042.58");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1393.18,4041.63");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2101.57,4143.95");
    }

}

