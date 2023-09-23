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

network DADCA_ACK_n50_u16_a8000m2_6
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
        @display("i=misc/sensor;p=1159.51,7979.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2629.45,7941.19");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3108.75,7943.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4065.50,7949.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4428.57,7924.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5059.62,7893.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5797.14,7981.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1988.68,7703.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2445.00,7550.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3079.70,7532.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4414.83,7505.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5180.94,7522.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=702.52,7481.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1296.60,7470.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1900.52,7344.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2793.18,7307.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3627.56,7479.12");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4765.40,7362.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5914.79,7443.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=906.48,7111.11");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1638.45,7013.57");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3160.83,7065.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3534.98,7013.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3921.11,7075.26");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4462.39,7116.70");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5148.48,7042.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=407.17,6946.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1218.02,6900.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2223.23,6893.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4802.06,6986.12");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5627.07,6980.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=125.20,6731.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1353.51,6525.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1704.92,6535.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4670.27,6557.07");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5255.97,6564.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5808.80,6644.95");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=409.89,6373.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2240.40,6316.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2655.87,6478.81");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3337.45,6439.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3720.85,6413.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4182.09,6481.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=803.59,6150.91");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3484.22,6116.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4830.03,6186.19");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5525.21,6208.52");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=128.00,5879.08");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1022.92,5790.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1487.98,5990.79");
    }

}

