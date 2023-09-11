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

network DADCA_ACK_n20_a8000m2_lpnFn2_1
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
        @display("i=misc/sensor,p=15.94,65.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=141.05,150.86");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=211.07,110.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=140.32,249.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=172.12,317.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=94.50,308.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=235.30,353.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=279.29,144.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=336.46,191.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=58.91,167.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=9.78,131.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=220.36,196.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=264.69,71.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=223.38,279.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=150.99,409.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=10.83,327.06");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=227.75,423.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=335.88,86.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=374.21,275.06");
    }

}

