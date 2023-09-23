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
        @display("i=misc/sensor;p=1569.72,7860.05");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3436.63,7939.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4803.60,7970.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=174.21,7514.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3200.30,7610.83");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3894.57,7657.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4517.95,7690.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5234.55,7731.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=593.61,7273.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1223.81,7413.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1589.45,7322.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2221.28,7314.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4276.34,7374.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4800.64,7454.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5429.92,7271.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2752.44,7232.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3249.95,7132.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3763.58,7224.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5876.62,7030.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=510.82,6920.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1152.59,6943.53");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1979.58,6841.68");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2544.44,6819.33");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3832.14,6855.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4618.31,6911.06");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5078.53,6973.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1185.35,6572.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3382.95,6534.22");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5322.46,6588.62");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5664.30,6671.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=127.91,6306.91");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2062.36,6414.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5029.89,6346.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5862.70,6310.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=446.44,6155.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1021.50,6043.08");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1542.29,6156.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2223.79,6067.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2629.84,6218.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3963.18,6246.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4505.58,6259.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5443.42,6075.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=312.43,5816.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3129.41,5795.60");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3489.57,5926.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2.15,5629.25");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=869.99,5658.97");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1544.40,5611.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1905.52,5748.44");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3894.13,5749.59");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4313.87,5729.69");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4725.10,5728.30");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5135.32,5697.51");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5779.91,5647.46");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=339.18,5349.05");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=691.44,5315.76");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2155.82,5397.64");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2684.71,5371.43");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3329.45,5497.35");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5449.09,5489.48");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5972.01,5326.50");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1084.00,5252.81");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1877.05,5109.09");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3932.54,5161.10");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4598.00,5179.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4980.72,5055.40");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5632.18,5155.53");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=20.85,4905.33");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=455.26,4885.20");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=913.88,4937.19");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1473.35,4866.68");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2345.38,4909.24");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3374.11,4936.98");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4324.34,4852.11");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5439.87,4851.48");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1783.35,4621.47");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2735.09,4724.97");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3825.81,4713.51");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4932.78,4682.86");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5805.82,4562.76");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=122.60,4401.87");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=561.79,4476.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1157.48,4413.53");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2185.11,4368.53");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2643.74,4310.42");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3407.44,4474.98");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3931.61,4340.81");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4586.47,4501.02");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5206.86,4318.41");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=705.84,4046.11");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3083.86,4049.83");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4808.82,4175.58");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=299.04,4004.60");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1502.41,3813.07");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2319.33,3822.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2688.54,3911.53");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3508.63,3873.69");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4408.96,3947.33");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5293.55,3925.75");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5656.12,3962.07");
    }

}

