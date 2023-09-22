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

network DADCA_ACK_n20_a8000m2_9
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
        @display("i=misc/sensor;p=306.62,7964.31");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1110.89,7789.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1740.22,7895.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2455.55,7943.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3519.96,7991.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3889.45,7991.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4665.49,7905.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=56.21,7624.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1532.95,7560.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2168.02,7631.83");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2739.80,7694.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3298.14,7631.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3989.48,7579.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5018.44,7748.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5504.90,7596.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=392.03,7482.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=841.11,7473.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2987.11,7419.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3595.85,7396.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4668.65,7383.96");
    }

}

