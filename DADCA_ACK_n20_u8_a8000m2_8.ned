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

network DADCA_ACK_n20_u8_a8000m2_8
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
        @display("i=misc/sensor;p=846.97,7755.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4063.59,7881.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4478.83,7768.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=453.07,7526.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1256.19,7662.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1813.91,7581.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2522.85,7517.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2832.63,7748.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3267.98,7661.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3675.65,7623.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5059.68,7744.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5672.16,7744.80");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=892.18,7308.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1507.33,7375.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3875.77,7294.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4270.85,7443.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4709.23,7460.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5277.42,7427.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5932.98,7486.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=442.22,7100.13");
    }

}

