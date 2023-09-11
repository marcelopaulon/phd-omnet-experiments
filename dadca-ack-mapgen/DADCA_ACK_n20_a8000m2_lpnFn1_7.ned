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

network DADCA_ACK_n20_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=31.37,97.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=85.67,173.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=43.70,17.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=174.50,153.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=228.25,191.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=299.68,138.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=311.64,236.20");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=266.56,322.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=334.31,294.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=183.08,270.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=360.39,78.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=419.39,32.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=452.48,108.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=417.50,159.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=467.69,241.52");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=404.32,274.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=227.60,94.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=302.35,404.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=253.39,482.54");
    }

}

