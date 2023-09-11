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
        @display("i=misc/sensor,p=30.26,65.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=36.27,162.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=191.89,18.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=171.74,87.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=162.77,148.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=38.18,253.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=120.84,239.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=59.83,5.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=128.78,299.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=213.81,257.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=24.40,350.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=10.87,449.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=16.31,514.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=57.21,569.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=48.57,651.51");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=21.16,743.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=159.56,648.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=100.21,701.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=180.06,554.15");
    }

}

