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

network DADCA_ACK_n100_u2_a8000m2_8
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
        @display("i=misc/sensor;p=332.27,7863.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=778.08,7960.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1363.75,7845.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1886.29,7812.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2856.19,7953.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3551.68,7976.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3829.69,7761.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4176.58,7916.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4555.14,7857.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4901.75,7926.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=656.53,7622.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2252.52,7682.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5517.46,7547.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=55.07,7423.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=955.17,7413.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1601.22,7447.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2116.58,7281.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2671.97,7409.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3190.24,7497.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3746.10,7331.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4588.38,7413.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5084.96,7427.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1339.28,7203.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1764.26,7047.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2892.17,7036.37");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4373.55,7053.44");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5970.45,7080.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=353.03,6989.93");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=858.65,6954.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2395.34,6821.78");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3425.72,6770.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3994.35,6850.03");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4720.59,6808.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5103.38,6990.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5533.26,7007.71");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4.58,6742.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1156.59,6528.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5327.98,6687.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5725.76,6628.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=440.15,6402.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1852.33,6471.91");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2660.66,6444.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3217.19,6401.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3589.32,6397.39");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4469.63,6514.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5037.68,6297.43");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4.80,6115.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1029.16,6163.51");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1376.97,6041.63");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4179.20,6239.10");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4563.86,6121.69");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5402.20,6175.54");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5895.27,6225.75");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=428.12,5938.76");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1802.42,5859.81");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2301.19,5967.93");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2948.45,5889.67");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3312.57,5815.68");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5896.49,5803.77");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=153.58,5545.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=915.08,5686.21");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1512.46,5573.04");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2263.71,5603.90");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3835.49,5727.02");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4219.86,5589.70");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4573.69,5649.65");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5011.46,5707.79");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5377.67,5770.27");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=687.67,5378.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2021.28,5330.77");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2607.86,5467.72");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3818.57,5319.06");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5177.12,5388.40");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5527.58,5450.22");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=311.98,5092.43");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1315.15,5243.53");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2839.08,5122.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3250.59,5196.52");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4441.06,5271.51");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4866.68,5149.66");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=969.26,4933.40");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3090.84,4833.35");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3679.98,4798.14");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5371.97,4993.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5733.06,4954.85");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=686.46,4575.74");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1352.75,4653.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1792.92,4724.68");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2268.07,4696.38");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4203.03,4750.86");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5824.11,4609.25");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=159.19,4351.10");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1617.52,4400.53");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2037.70,4374.11");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2460.02,4314.41");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3716.83,4445.96");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4432.79,4389.03");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4881.27,4529.93");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5335.76,4433.22");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=907.84,4201.50");
    }

}

