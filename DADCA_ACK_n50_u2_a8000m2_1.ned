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

network DADCA_ACK_n50_u2_a8000m2_1
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
        @display("i=misc/sensor;p=43.52,7932.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=554.86,7947.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1197.73,7900.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2769.57,7886.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3389.49,7812.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4021.15,7919.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5663.08,7815.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=401.23,7514.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=791.52,7652.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1251.02,7554.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2368.62,7556.88");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2746.70,7529.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3121.91,7558.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5124.79,7584.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3990.16,7472.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4436.50,7457.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1465.55,7080.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3248.60,7067.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3716.21,7234.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4133.23,7134.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4690.87,7105.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5246.28,7147.86");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5606.97,7180.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=297.23,6956.59");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=685.07,6939.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2326.14,6846.21");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2721.71,6992.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3522.51,6794.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4067.17,6783.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=231.25,6603.16");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=732.96,6588.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1427.23,6634.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2851.80,6615.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4528.93,6719.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5104.42,6681.54");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5483.85,6626.80");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5944.66,6757.90");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1090.43,6497.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2036.95,6468.13");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2475.28,6335.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3204.19,6414.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3639.27,6425.84");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4009.78,6385.32");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4356.04,6300.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4766.04,6348.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=731.72,6030.84");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1574.21,6087.50");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2807.39,6167.10");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5252.81,6084.87");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5863.02,6120.20");
    }

}

