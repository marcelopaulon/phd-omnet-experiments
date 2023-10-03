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

network DADCA_ACK_n100_u2_a8000m2_0
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
        @display("i=misc/sensor;p=752.66,7775.47");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1154.65,7947.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2629.20,7931.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3619.02,7764.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4022.07,7960.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4633.46,7935.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5226.02,7923.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=166.04,7559.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1534.24,7654.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2148.66,7531.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4118.58,7611.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5705.30,7745.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=949.84,7335.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2659.69,7342.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5123.43,7427.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=568.53,7250.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1549.12,7207.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3079.22,7073.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3412.23,7238.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4635.47,7169.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=315.15,6981.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1195.13,6981.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2188.10,6868.92");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4064.85,6951.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4955.21,6818.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5512.66,6915.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=249.31,6632.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=900.25,6719.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1619.54,6614.79");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2544.76,6536.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2952.82,6731.52");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3714.45,6662.07");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4147.71,6558.09");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4476.61,6738.81");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1267.67,6485.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3225.63,6461.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3573.89,6271.81");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4463.75,6325.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5145.32,6372.44");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=83.49,6218.51");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=474.05,6221.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1013.36,6186.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1425.77,6056.43");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1829.08,6178.39");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2228.67,6178.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2722.72,6207.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3062.15,6105.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4848.53,6076.26");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5502.36,6129.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=221.98,5866.96");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3925.71,5980.41");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=726.47,5742.16");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1340.53,5711.71");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1708.00,5608.93");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2320.86,5534.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2871.49,5650.29");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3509.60,5660.49");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4201.72,5724.62");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4656.79,5688.35");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5105.04,5643.28");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5948.16,5763.05");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=178.31,5316.77");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4013.23,5419.15");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4559.97,5335.12");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4926.84,5302.82");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5522.88,5506.14");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=562.92,5263.80");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1105.80,5262.83");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1747.21,5143.58");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2121.34,5169.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2928.63,5208.76");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3436.10,5275.15");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3737.27,5067.23");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5266.36,5193.65");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5935.75,5258.39");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=66.51,4795.44");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=670.81,4791.53");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1219.78,4892.98");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2454.26,4975.59");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4198.28,4804.67");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5613.91,4800.59");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1201.59,4536.29");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1762.76,4669.04");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2211.20,4651.49");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2896.95,4745.92");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4754.03,4599.84");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5164.31,4775.27");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4.77,4405.05");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2771.46,4406.48");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3610.10,4458.82");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4254.06,4288.15");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5945.96,4302.54");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=421.49,4148.56");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=814.82,4087.67");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1822.48,4100.25");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2286.92,4226.02");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3888.23,4236.64");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5151.46,4183.76");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=82.37,3877.01");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=550.01,3795.22");
    }

}

