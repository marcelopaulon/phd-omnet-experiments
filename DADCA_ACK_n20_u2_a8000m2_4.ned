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

network DADCA_ACK_n20_u2_a8000m2_4
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
        @display("i=misc/sensor;p=16.76,7870.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=682.23,7811.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2053.65,7827.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3636.45,7944.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4186.09,7758.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4933.40,7818.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5607.34,7871.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1021.52,7597.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1664.13,7659.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2845.52,7533.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5962.21,7566.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=609.68,7437.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3281.88,7282.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3930.10,7470.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4739.63,7320.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5152.78,7439.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=58.58,7071.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=950.07,7071.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1479.18,7110.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2160.31,7180.09");
    }

}

