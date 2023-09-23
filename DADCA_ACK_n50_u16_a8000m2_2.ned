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

network DADCA_ACK_n50_u16_a8000m2_2
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
        @display("i=misc/sensor;p=474.68,7969.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1765.50,7934.73");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2738.31,7940.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3459.39,7988.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3865.79,7890.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4595.53,7851.75");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=822.63,7627.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1474.25,7623.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2369.39,7739.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3180.15,7670.69");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4074.94,7558.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5067.64,7653.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5928.35,7665.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=24.80,7416.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=486.73,7348.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1859.28,7355.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2297.00,7281.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2645.13,7382.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3494.95,7438.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4601.16,7301.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5470.78,7444.46");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=717.76,7082.02");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1332.16,7245.72");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3074.94,7199.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3593.56,7050.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5387.40,7070.40");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5838.88,7125.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=177.10,6805.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1420.01,6854.24");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2055.97,6918.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2711.84,6857.20");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4119.00,6965.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4905.54,6959.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=867.55,6725.47");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1692.12,6587.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2389.24,6637.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3057.63,6556.63");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4684.67,6617.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5441.25,6607.87");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5824.57,6663.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=77.22,6378.04");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1029.53,6327.76");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2293.55,6298.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2659.81,6359.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3694.12,6514.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4212.14,6425.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5069.23,6351.32");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=436.79,6224.81");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1487.49,6198.94");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3620.49,6060.17");
    }

}

