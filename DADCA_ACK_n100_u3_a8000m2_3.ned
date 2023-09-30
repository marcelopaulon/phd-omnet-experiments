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

network DADCA_ACK_n100_u3_a8000m2_3
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
        @display("i=misc/sensor;p=499.14,7800.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1024.30,7982.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1696.44,7992.95");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2157.53,7782.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2677.93,7907.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3574.46,7918.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5897.59,7760.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1290.99,7657.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3108.09,7728.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4021.06,7552.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5214.69,7674.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=183.79,7366.58");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=881.66,7405.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2180.12,7342.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2929.07,7410.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3551.82,7498.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4719.36,7399.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1258.73,7239.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1788.02,7237.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2586.28,7158.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3461.91,7157.66");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4115.28,7035.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5229.12,7164.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5589.09,7249.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=512.14,6958.07");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1344.99,6808.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1860.24,6878.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3143.15,6777.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=248.52,6711.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=991.39,6658.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2228.35,6745.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2585.17,6629.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3589.99,6642.51");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4447.57,6522.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5156.94,6742.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5579.91,6617.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1613.00,6441.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2886.72,6296.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=997.08,6081.62");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2107.05,6178.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3316.70,6068.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3883.47,6209.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4884.06,6232.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5329.88,6060.29");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5740.73,6076.27");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=470.51,6014.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1414.81,5846.38");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1864.18,5897.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2345.01,5877.02");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2759.85,5908.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4605.61,5850.21");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=464.86,5641.79");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1751.37,5544.57");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2225.23,5544.33");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3184.42,5736.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3963.15,5729.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5055.78,5733.19");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5491.18,5607.30");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5943.84,5532.86");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=155.93,5302.11");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=980.57,5499.56");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1395.73,5280.00");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2769.53,5452.81");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3525.94,5491.32");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4047.02,5380.59");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4582.65,5388.30");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=507.37,5212.57");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3252.28,5172.76");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5256.03,5120.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1163.95,4988.58");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1818.52,4970.27");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2413.74,4952.32");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2776.26,4918.99");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3688.47,4818.96");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4019.52,5022.33");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4588.16,4926.24");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=223.00,4622.78");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=717.39,4740.82");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1398.00,4675.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2019.52,4665.63");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3087.81,4696.87");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4263.14,4743.94");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4891.82,4696.79");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5330.08,4537.49");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1829.70,4371.09");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2319.82,4355.52");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2741.32,4473.47");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3326.14,4434.20");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3766.73,4312.14");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5783.37,4324.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=45.44,4263.81");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=680.37,4172.52");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1071.34,4145.07");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1441.77,4127.14");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5302.88,4077.01");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1246.58,3795.41");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1798.43,3885.39");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2463.67,3895.92");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2831.99,4013.32");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3352.59,3800.75");
    }

}

