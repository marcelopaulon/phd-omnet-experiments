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

network DADCA_ACK_n20_u4_a8000m2_6
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
        @display("i=misc/sensor;p=281.73,7965.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=860.85,7841.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2332.37,7813.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2781.34,7792.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4045.26,7764.86");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4928.47,7909.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1720.39,7704.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3171.69,7564.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3559.93,7605.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4491.70,7691.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5424.15,7713.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5822.17,7732.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=73.99,7410.39");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=430.83,7362.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1871.31,7351.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2276.56,7399.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5731.49,7269.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=154.71,7047.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=726.25,7080.99");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1198.13,7191.33");
    }

}

