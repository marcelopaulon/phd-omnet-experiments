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

network DADCA_ACK_n100_u2_a8000m2_4
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
        @display("i=misc/sensor;p=912.08,7944.10");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1351.96,7906.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1954.01,7961.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4222.56,7856.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5362.82,7886.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=80.39,7555.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=497.79,7741.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1665.21,7640.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2641.17,7677.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3071.39,7735.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3529.47,7738.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4497.86,7591.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4878.53,7643.24");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=415.58,7371.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=960.81,7401.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1389.65,7351.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2072.87,7463.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2977.19,7362.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4039.95,7427.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5146.46,7348.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5532.64,7312.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=188.31,7082.86");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1814.25,7092.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2335.75,7120.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3412.67,7133.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3844.05,7081.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4457.48,7165.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5956.31,7212.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=765.31,6972.84");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1381.61,6800.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2958.64,6897.47");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4270.16,6813.02");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4725.65,6776.70");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5075.75,6798.98");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5578.08,6938.50");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=513.77,6615.89");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1935.67,6682.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2376.55,6607.40");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3514.81,6570.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5825.34,6687.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1032.13,6337.05");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3045.40,6446.80");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3976.96,6484.15");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5230.96,6313.07");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=238.21,6259.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=579.02,6145.94");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1561.85,6168.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2705.50,6025.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4632.53,6214.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5751.73,6049.80");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1127.76,5905.10");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1881.55,5825.94");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2232.49,6009.01");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3367.14,5869.78");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4081.78,6001.57");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5244.77,5952.05");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=274.80,5597.32");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=826.48,5629.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2908.11,5732.14");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4545.89,5687.34");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5599.06,5560.04");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1341.20,5506.34");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1907.49,5352.17");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2326.80,5478.21");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2725.68,5415.41");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3244.76,5308.98");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3712.79,5404.73");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4086.69,5402.74");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4845.77,5283.25");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5174.97,5507.17");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=495.16,5238.81");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1492.60,5187.81");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2235.30,5073.21");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4440.39,5082.54");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5162.54,5118.42");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5527.35,5113.56");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=716.35,4901.07");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1456.75,4810.57");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1907.27,4870.93");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2557.84,4879.68");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3056.63,4970.48");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3522.96,4911.04");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5782.61,4791.90");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2404.15,4561.66");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2992.49,4567.33");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5397.54,4599.00");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=64.46,4442.47");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=717.98,4299.93");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2163.35,4298.19");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3600.75,4402.70");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4280.24,4506.16");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5044.93,4525.14");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5668.41,4370.96");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1163.94,4234.25");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1538.72,4284.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2559.38,4210.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2986.25,4168.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4731.00,4251.49");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5081.50,4146.27");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5462.38,4055.51");
    }

}

