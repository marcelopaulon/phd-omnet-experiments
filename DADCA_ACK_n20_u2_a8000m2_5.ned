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

network DADCA_ACK_n20_u2_a8000m2_5
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
        @display("i=misc/sensor;p=1893.65,7921.22");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3103.08,7831.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3508.16,7840.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4031.12,7776.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4605.47,7880.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=282.07,7611.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=973.48,7514.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1433.21,7616.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2245.04,7696.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2741.40,7669.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4860.90,7618.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5964.53,7644.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1839.38,7380.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3236.51,7412.11");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3619.82,7445.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4096.69,7350.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4478.16,7393.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5424.45,7471.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1057.71,7112.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1682.43,7020.71");
    }

}

