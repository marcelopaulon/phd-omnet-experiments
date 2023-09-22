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

network DADCA_ACK_n100_a8000m2_9
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
        @display("i=misc/sensor;p=301.55,7984.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1923.39,7991.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3511.40,7812.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3875.72,7977.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4400.61,7863.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5011.88,7937.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=240.13,7528.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=675.82,7536.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1188.76,7594.11");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2086.46,7646.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4763.47,7639.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5674.65,7604.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2451.62,7427.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3778.50,7355.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4129.14,7306.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5201.10,7461.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=484.34,7012.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=891.02,7010.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1503.30,7131.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1906.93,7082.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3349.72,7173.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4717.10,7010.37");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2423.98,6954.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2875.56,6878.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3739.37,6835.27");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4249.62,6892.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5334.12,6888.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=869.05,6620.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1852.50,6562.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2709.09,6567.26");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3366.00,6751.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4738.67,6579.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5234.24,6533.25");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=491.65,6439.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1501.90,6492.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2358.66,6348.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3189.30,6436.51");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3755.66,6444.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4278.81,6282.07");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5613.03,6414.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=726.23,6082.60");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2636.00,6030.93");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3539.92,6137.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=153.27,5992.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1370.90,5985.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1868.98,5965.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4065.94,5957.28");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4549.13,6019.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5159.28,6000.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5651.11,5797.36");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=333.41,5597.33");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2095.81,5533.06");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2977.64,5580.89");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3723.67,5686.43");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=785.43,5523.41");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1130.85,5419.49");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4522.68,5523.58");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5118.22,5508.80");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=168.79,5060.06");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1650.47,5102.81");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2180.26,5161.75");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2570.28,5121.01");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3795.85,5044.41");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4333.23,5126.35");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5038.46,5066.34");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5489.40,5213.12");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=496.30,4873.92");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=974.77,4837.76");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3193.00,5014.32");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4711.27,4806.81");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5722.86,4830.68");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1403.19,4538.52");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2162.12,4691.35");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2742.12,4630.09");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4060.81,4657.72");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5102.98,4573.70");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=347.13,4432.12");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2087.11,4322.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3230.88,4495.48");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3585.03,4421.22");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5783.52,4312.42");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=916.78,4260.32");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1692.74,4268.75");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2408.08,4163.90");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3174.55,4135.32");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4193.21,4249.75");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5391.06,4072.72");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=279.65,3932.15");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1086.97,3941.83");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2002.60,3819.44");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2434.03,3798.54");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3750.03,3875.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4279.92,3906.80");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4824.77,3989.64");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=85.70,3624.96");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=678.40,3584.57");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1644.27,3633.29");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2858.29,3652.89");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3458.20,3598.02");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5063.45,3701.21");
    }

}

