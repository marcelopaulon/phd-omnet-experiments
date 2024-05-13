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

network DADCA_ACK_n100_u4_a8000m2_7
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
        @display("i=misc/sensor;p=739.72,7937.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2491.58,7776.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3498.52,7874.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4082.73,7939.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4452.55,7976.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=284.07,7581.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1221.81,7574.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1881.21,7645.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3047.06,7712.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3996.79,7559.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4776.45,7684.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1692.30,7299.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2906.96,7336.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3447.98,7376.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4496.93,7443.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=401.84,7218.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=952.36,7198.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1297.52,7093.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2312.08,7192.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3209.75,7111.00");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5199.88,7136.91");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5667.19,7024.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=81.87,6779.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=870.04,6806.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1952.21,6780.50");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2809.42,6787.14");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3540.55,6853.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4124.05,6906.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4649.91,6803.33");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=620.46,6549.13");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2363.49,6720.47");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4353.38,6592.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5325.21,6656.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5962.06,6737.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1112.67,6514.47");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1556.36,6484.04");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2772.09,6392.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3674.46,6493.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4983.81,6444.40");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5664.16,6418.90");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=439.03,6081.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2178.02,6138.09");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3342.85,6078.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4036.71,6262.95");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5305.47,6254.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5959.35,6164.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=88.48,5868.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1032.81,5867.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1703.23,5849.73");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2908.18,5933.71");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3895.67,5938.67");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4672.78,5991.16");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5602.39,5967.00");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=697.54,5700.26");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2599.62,5724.99");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3295.55,5668.12");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3741.83,5525.81");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4150.82,5671.03");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4530.85,5651.28");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5127.73,5627.97");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5891.60,5708.91");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1493.25,5378.19");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2215.20,5479.94");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2608.28,5279.71");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5258.72,5278.04");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5602.23,5400.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=107.37,5232.83");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=484.92,5219.56");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1165.89,5127.25");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1817.58,5175.43");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2232.06,5091.75");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3149.08,5191.61");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3546.61,5096.99");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4084.31,5122.67");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4500.66,5201.73");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4845.39,5043.12");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5864.52,5053.58");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=25.99,4835.88");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=906.51,4832.03");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2615.06,4884.62");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5346.46,4877.93");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1438.88,4775.27");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3199.06,4779.99");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3650.61,4579.60");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4117.88,4663.40");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5128.12,4592.47");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=313.29,4520.81");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=772.39,4432.39");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1473.17,4305.66");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1889.20,4523.84");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2510.85,4472.27");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3029.02,4465.04");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4852.54,4366.59");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5874.19,4452.42");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=440.53,4159.67");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2042.53,4164.21");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2818.74,4136.93");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3337.15,4230.88");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3711.19,4162.46");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4187.02,4280.84");
    }

}

