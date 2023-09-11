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

network DADCA_ACK_n20_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=9.84,118.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=171.40,68.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=230.75,112.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=114.43,162.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=179.43,154.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=48.23,66.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=74.44,0.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=37.31,182.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=251.74,178.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=310.29,155.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=171.44,218.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=282.91,80.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=255.37,244.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=298.80,289.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=390.06,323.56");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=332.49,346.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=379.07,386.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=489.66,417.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=572.13,455.95");
    }

}

