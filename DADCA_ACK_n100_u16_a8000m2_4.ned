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
        @display("i=misc/sensor;p=624.70,7785.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1459.99,7898.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1982.37,7959.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2717.60,7805.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3358.57,7961.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4586.28,7907.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5626.27,7855.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1625.80,7556.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2440.12,7575.44");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3189.60,7562.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4334.71,7570.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4979.13,7641.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=835.88,7281.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1189.56,7445.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2638.24,7263.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3640.84,7501.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1413.34,7061.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2149.34,7213.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3452.06,7126.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3952.79,7018.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4368.41,7187.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4698.77,7028.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5336.40,7250.12");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5781.42,7244.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1797.71,6956.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2673.40,6765.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3017.17,6896.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4406.97,6802.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5039.90,6897.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=229.20,6688.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=954.62,6612.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2274.64,6729.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3297.35,6570.64");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3690.15,6723.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5498.67,6574.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5973.86,6580.33");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=756.25,6302.56");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1326.59,6356.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1672.31,6514.32");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1992.81,6347.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2636.52,6381.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3830.48,6312.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4121.04,6514.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4792.92,6442.12");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3423.03,6201.41");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4486.85,6163.35");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5166.54,6060.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5547.26,6072.73");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5994.05,6185.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=301.25,5949.21");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=725.88,5899.63");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1687.12,5987.64");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2103.49,5973.27");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2498.03,6009.36");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3177.98,5915.11");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4120.20,5885.42");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1498.67,5649.23");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2373.24,5571.82");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3518.45,5640.81");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4746.28,5698.26");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5280.78,5575.33");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5633.64,5709.14");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=69.34,5297.01");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=447.32,5406.01");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1901.67,5424.34");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2709.38,5444.20");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3193.66,5411.85");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3847.08,5457.10");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5766.87,5281.02");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1098.04,5173.38");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2163.57,5052.75");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4577.94,5219.76");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=555.23,5012.44");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1309.02,4887.51");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1807.34,5014.29");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2540.75,5000.00");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2918.89,5013.01");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3492.06,4874.89");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3983.98,4934.01");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4908.61,4786.53");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=806.28,4742.05");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1221.00,4540.61");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1882.47,4562.88");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2784.22,4559.63");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3208.72,4624.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4450.47,4701.25");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5547.30,4671.44");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=324.35,4432.34");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2266.97,4493.54");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3939.14,4435.42");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5192.44,4358.68");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1090.13,4166.52");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1444.68,4048.71");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1801.83,4208.22");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2139.66,4103.12");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2745.06,4121.94");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3590.26,4225.53");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4185.47,4084.02");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4516.22,4240.58");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=342.69,3837.37");
    }

}

