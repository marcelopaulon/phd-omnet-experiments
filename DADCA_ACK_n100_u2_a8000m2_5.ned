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

network DADCA_ACK_n100_u2_a8000m2_5
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
        @display("i=misc/sensor;p=966.71,7870.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1627.95,7984.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2083.89,7980.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3144.49,7997.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3891.00,7799.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4415.90,7939.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=456.11,7605.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1292.99,7620.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1792.92,7659.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2541.94,7618.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2920.72,7564.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3544.48,7727.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4329.64,7587.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5008.93,7567.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5525.24,7729.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=586.80,7268.40");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1016.01,7289.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2076.96,7297.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3466.69,7384.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5423.92,7300.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5990.68,7463.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=146.04,7011.12");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1668.81,7056.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2536.54,7179.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2920.56,7082.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3720.23,7047.36");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4360.44,7104.42");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5792.28,7165.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=548.09,6777.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1045.43,6869.06");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2277.56,6840.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4826.68,6925.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2530.35,6572.06");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3263.81,6707.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5505.74,6678.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=178.77,6316.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=567.96,6405.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1364.50,6424.89");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1964.57,6459.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2862.39,6352.62");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3709.77,6377.18");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4108.86,6434.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4482.25,6479.42");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5007.61,6283.27");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5912.50,6505.66");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=868.06,6196.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1941.36,6079.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2365.04,6225.26");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3411.47,6123.22");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3968.49,6076.88");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5541.46,6025.57");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5960.84,6031.41");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=665.41,5839.15");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1340.71,5783.89");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3209.46,5834.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4590.40,5812.19");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5211.93,5858.01");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=234.55,5661.39");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2366.88,5559.83");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2714.54,5665.31");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3899.98,5692.58");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5637.24,5591.17");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2.00,5326.24");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=982.90,5479.54");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3044.39,5402.76");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3662.96,5360.37");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4521.80,5445.91");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5084.27,5374.31");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=412.73,5196.82");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1225.15,5106.38");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1699.94,5208.61");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2518.80,5047.75");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5495.66,5067.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1015.41,4789.65");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1701.58,4834.25");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2965.09,4840.31");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3277.66,5020.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3870.15,5006.07");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4498.04,4826.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5055.20,5008.46");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=442.73,4736.93");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1375.05,4617.36");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2087.27,4753.36");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3947.69,4658.63");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=184.87,4327.41");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=859.95,4349.27");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3423.95,4468.36");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5265.93,4527.35");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1641.57,4138.92");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2457.51,4218.89");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3104.73,4219.98");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3761.43,4218.54");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4297.03,4071.11");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5482.23,4054.66");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=527.82,3927.59");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2133.83,4005.56");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3989.28,3795.88");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4617.62,3873.04");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5069.78,3934.71");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=150.31,3664.16");
    }

}

