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

network DADCA_ACK_n20_u8_a8000m2_5
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
        @display("i=misc/sensor;p=294.39,7964.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1781.28,7798.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2776.13,7856.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3191.01,7852.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4092.92,7950.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4769.57,7978.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5129.39,7905.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5845.65,7930.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=840.78,7676.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2281.38,7735.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3667.67,7706.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4241.18,7624.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5444.35,7689.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=327.34,7438.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1698.29,7278.79");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2103.83,7430.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2478.72,7357.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3163.96,7485.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3985.48,7267.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4840.21,7486.77");
    }

}

