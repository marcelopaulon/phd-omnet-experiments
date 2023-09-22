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

network DADCA_ACK_n20_a8000m2_3
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
        @display("i=misc/sensor;p=185.45,7993.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=282.51,7986.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=551.40,7999.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=616.50,7972.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=750.40,7980.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=965.42,7967.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=17.95,7934.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=212.93,7936.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=302.99,7927.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=461.74,7942.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=524.98,7924.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=688.15,7955.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=849.10,7931.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=108.82,7909.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=362.26,7883.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=644.49,7901.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=886.04,7876.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=10.16,7841.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=219.52,7862.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=282.19,7868.15");
    }

}

