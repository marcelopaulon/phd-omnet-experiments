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

network DADCA_ACK_n100_a8000m2_6
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
        @display("i=misc/sensor;p=610.10,7825.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1053.59,7782.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1811.72,7953.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2822.15,7862.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3383.16,7832.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4128.56,7963.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5864.66,7990.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=336.12,7527.84");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3956.22,7550.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4756.92,7666.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5189.98,7723.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5688.65,7632.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=821.79,7487.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1305.97,7495.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1712.60,7456.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3222.86,7359.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4352.53,7477.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1037.26,7201.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1459.91,7047.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2175.11,7210.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2709.31,7154.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3713.35,7137.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4879.49,7146.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5378.20,7196.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=442.00,6912.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2484.76,6766.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3310.24,6816.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3943.92,6772.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4527.72,7008.74");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5178.08,6829.79");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5656.01,6960.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1114.23,6651.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1932.80,6720.56");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2851.85,6667.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=307.84,6311.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1465.75,6442.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3447.38,6476.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3796.59,6450.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4208.46,6409.76");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5432.58,6478.95");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5874.87,6282.47");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=827.35,6232.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1210.50,6129.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2075.88,6114.65");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2628.65,6185.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3683.78,6070.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4669.93,6261.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5150.93,6249.01");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5450.25,6049.90");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=196.32,5805.35");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=684.35,5890.66");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1729.34,5823.25");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3217.08,5952.77");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4036.98,6004.23");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4908.99,5956.61");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1364.51,5595.13");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2140.08,5554.31");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2714.04,5542.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3741.52,5622.32");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4462.32,5600.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5263.13,5740.41");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5636.09,5702.79");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=472.24,5302.74");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=939.36,5366.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3359.49,5462.64");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4174.26,5334.30");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=733.03,5056.87");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1546.62,5219.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2116.22,5075.20");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3013.56,5191.62");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4843.82,5236.09");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5313.21,5221.36");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5656.55,5096.49");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2573.76,5029.52");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3331.25,4820.10");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3850.61,4889.25");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4330.89,5012.66");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5976.94,4797.90");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=80.34,4569.67");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=455.32,4686.56");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=913.53,4684.04");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1317.10,4687.81");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2893.13,4558.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4415.78,4666.77");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5184.00,4751.84");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2055.68,4438.56");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2402.96,4492.37");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3596.26,4472.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3940.09,4353.49");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5600.45,4412.87");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5980.37,4304.84");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=200.89,4143.91");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=646.72,4183.23");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1139.78,4171.13");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1666.30,4287.61");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2421.03,4085.21");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3010.40,4114.27");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3696.03,4091.06");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4813.51,4146.43");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5217.00,4246.12");
    }

}

