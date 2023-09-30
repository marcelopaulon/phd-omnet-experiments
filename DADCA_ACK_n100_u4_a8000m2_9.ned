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

network DADCA_ACK_n100_u4_a8000m2_9
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
        @display("i=misc/sensor;p=609.96,7883.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1022.95,7951.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1467.45,7808.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1841.66,7944.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2769.23,7971.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3293.73,7966.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3715.75,7910.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4808.85,7772.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5662.52,7824.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1167.82,7625.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2155.00,7656.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2719.86,7569.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4390.04,7746.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5282.39,7709.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=515.64,7473.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1043.98,7277.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1470.13,7449.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3268.18,7474.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3926.92,7498.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4477.39,7294.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5500.61,7274.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=55.46,7218.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=455.26,7116.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1777.38,7199.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3623.74,7236.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4126.64,7206.56");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4856.54,7253.46");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5855.71,7230.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1325.90,7007.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2319.60,6987.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2686.21,6790.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3234.36,6838.21");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4010.17,6764.14");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4410.46,6900.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4778.81,6882.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5783.20,6858.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=430.31,6532.99");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=887.24,6611.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2290.81,6573.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3662.36,6672.60");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5349.69,6561.87");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1341.48,6498.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1825.41,6504.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3178.63,6424.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3833.41,6299.41");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4505.73,6510.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4927.39,6478.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=438.14,6030.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1480.67,6096.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2082.05,6262.44");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2781.23,6198.11");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3339.06,6100.99");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4416.16,6080.32");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5673.69,6235.98");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=62.96,6016.92");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1125.74,5941.06");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1806.27,5872.38");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2375.78,5998.73");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2775.48,5808.88");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3740.31,5906.41");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4773.24,5946.63");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5220.10,5951.68");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=901.88,5622.59");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2304.70,5604.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3321.46,5540.01");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4421.34,5565.88");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4906.91,5564.28");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5471.60,5688.11");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=211.52,5376.80");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1301.13,5473.10");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3812.85,5278.20");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5904.92,5425.14");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=551.60,5097.67");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1041.53,5151.17");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1775.80,5216.68");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2136.07,5106.81");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2725.09,5086.40");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4869.66,5152.18");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5326.07,5189.58");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=54.21,4840.82");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=872.26,4829.93");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1445.22,4876.96");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3147.00,5003.57");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4262.49,4840.65");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5835.48,4799.16");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=422.32,4710.16");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3377.38,4703.39");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3798.17,4684.81");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5184.02,4702.77");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=633.76,4305.06");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=982.31,4486.43");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1539.21,4395.64");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2020.32,4341.95");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2586.00,4450.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3081.20,4510.85");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3735.65,4301.72");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4088.18,4314.76");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4771.37,4385.23");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5829.97,4407.91");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=150.85,4287.58");
    }

}

