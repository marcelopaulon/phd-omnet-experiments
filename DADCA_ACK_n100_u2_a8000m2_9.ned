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

network DADCA_ACK_n100_u2_a8000m2_9
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
        @display("i=misc/sensor;p=11.55,7770.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=470.82,7928.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=869.26,7816.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1388.27,7936.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1791.97,7829.83");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2730.05,7974.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3161.83,7901.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3554.30,7953.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4033.80,7931.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4622.86,7823.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5256.97,7874.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1141.36,7580.99");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2335.10,7717.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2855.96,7630.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3775.24,7598.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4365.16,7571.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5626.57,7537.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=228.96,7486.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2299.69,7279.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3405.97,7309.31");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4048.60,7352.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5048.65,7408.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=41.83,7108.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=601.18,7177.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1667.51,7243.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4599.22,7117.93");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5228.52,7028.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5680.72,7114.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1810.98,6825.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2234.99,6770.56");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2902.57,7005.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3552.57,6899.78");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4078.83,6859.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5990.67,6789.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=216.62,6593.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=664.89,6652.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3960.36,6523.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5037.29,6714.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5581.47,6618.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=730.41,6293.81");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1252.32,6465.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1860.03,6316.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2540.96,6401.65");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3531.31,6276.60");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4440.26,6482.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4981.84,6366.14");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2220.93,6259.93");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3074.34,6262.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4133.94,6116.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=465.83,5978.41");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1583.94,6012.63");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2390.75,5828.67");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2942.91,5873.82");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3429.13,5773.49");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3888.97,5843.39");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4568.83,5935.09");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5116.42,5894.21");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5548.55,5838.65");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1075.61,5762.40");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1959.31,5683.46");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4782.13,5532.01");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=330.11,5443.23");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=835.32,5480.21");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1573.17,5354.28");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2895.02,5387.24");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3356.66,5397.17");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4022.35,5434.79");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4387.20,5337.22");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5366.93,5357.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5921.16,5295.29");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=711.21,5144.80");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1228.63,5173.58");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2164.88,5222.98");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3831.84,5106.81");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4856.36,5154.66");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1069.56,4821.21");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1635.83,4895.74");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3117.17,4907.82");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4257.82,4816.65");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5011.35,4797.85");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5478.48,5022.66");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=268.22,4619.86");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1358.04,4570.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2134.78,4629.22");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3675.17,4701.08");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5645.51,4556.98");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=681.06,4485.68");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1887.29,4296.17");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3221.57,4304.99");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4033.87,4441.37");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4635.11,4409.00");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5360.25,4300.94");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5888.21,4293.46");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2551.63,4199.16");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2912.50,4073.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4328.89,4199.63");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5102.63,4051.54");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=489.70,3837.16");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1037.09,4013.23");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1488.71,4018.54");
    }

}

