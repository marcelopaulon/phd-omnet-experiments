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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=161.22,71.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=237.53,113.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=161.58,133.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=210.62,34.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=284.49,61.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=313.89,221.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=357.92,144.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=434.86,206.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=460.96,126.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=455.81,45.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=379.61,88.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=221.53,238.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=142.90,274.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=72.93,242.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=84.83,171.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=128.40,10.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=311.28,344.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=408.52,335.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=446.43,385.08");
    }

}

