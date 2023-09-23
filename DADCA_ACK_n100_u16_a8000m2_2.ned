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

network DADCA_ACK_n100_u16_a8000m2_2
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
        @display("i=misc/sensor;p=1356.52,7821.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2696.26,7988.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3066.74,7955.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3453.22,7829.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4002.12,7833.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5127.88,7768.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5688.42,7897.29");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=123.88,7700.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=811.81,7564.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1769.28,7706.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2405.04,7584.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2829.37,7601.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4826.18,7588.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=508.15,7315.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1238.03,7439.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2053.09,7490.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3435.38,7432.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3806.87,7305.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4122.06,7477.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5382.69,7449.24");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5870.18,7399.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4.89,7049.49");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1099.24,7093.91");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1584.63,7224.25");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2011.11,7133.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2615.51,7156.16");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3478.03,7024.09");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4859.16,7077.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=576.20,6937.06");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1542.20,6789.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2389.61,6858.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3103.52,6968.02");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3853.92,6949.95");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4340.75,6930.49");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5747.28,6941.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=61.80,6622.45");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1951.42,6702.57");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2805.23,6699.69");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3990.29,6573.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4792.58,6558.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5438.01,6575.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5962.83,6626.61");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=781.94,6302.87");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1093.62,6489.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1487.32,6395.66");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2447.77,6430.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3335.15,6494.47");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5048.50,6291.91");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5784.70,6319.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=210.79,6200.34");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1725.82,6040.13");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2160.41,6082.60");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3841.21,6020.74");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4469.54,6097.88");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=927.59,5940.88");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1311.04,5997.01");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2862.20,6009.10");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5672.09,5965.34");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=328.92,5710.05");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1259.33,5583.33");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2482.12,5606.20");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3271.44,5640.94");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4201.49,5746.31");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4884.98,5701.39");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5628.54,5598.40");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=57.98,5327.79");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=768.49,5377.42");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1920.64,5521.14");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2695.21,5309.69");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3206.28,5289.10");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3589.12,5490.22");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4619.33,5426.39");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5214.30,5363.01");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1000.85,5043.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1385.86,5248.86");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2335.62,5139.71");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4239.30,5209.94");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5836.21,5094.93");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=33.31,4854.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=437.40,4987.06");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1787.42,4968.72");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2863.18,4994.67");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3745.23,5022.38");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4557.51,4877.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5034.60,5011.41");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5408.15,5011.96");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1352.18,4766.37");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1725.43,4619.40");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2096.39,4614.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2861.95,4636.00");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3751.21,4589.75");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4117.04,4736.46");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4840.30,4549.84");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=967.94,4379.60");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1485.87,4331.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3326.43,4465.19");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4215.59,4377.53");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5333.96,4424.95");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5772.41,4453.72");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1919.17,4060.56");
    }

}

