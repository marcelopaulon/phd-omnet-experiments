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

network DADCA_ACK_n20_u2_a8000m2_3
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
        @display("i=misc/sensor;p=1695.06,7943.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2766.91,7753.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4673.98,7809.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=713.65,7610.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1239.87,7606.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2269.05,7692.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3398.42,7649.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5212.52,7748.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5703.39,7722.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=339.82,7426.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1917.16,7504.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2346.98,7340.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2978.36,7260.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3731.15,7291.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4980.05,7466.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=75.45,7059.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=569.80,7027.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1158.07,7114.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1881.00,7076.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2713.84,7017.92");
    }

}

