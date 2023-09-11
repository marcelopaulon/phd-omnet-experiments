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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=163.99,58.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=256.50,54.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=251.77,120.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=78.54,185.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=159.05,127.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=144.62,193.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=182.41,242.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=51.42,271.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=29.20,339.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=13.93,405.98");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=98.25,524.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=28.25,581.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=41.71,489.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=113.74,654.26");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=179.39,686.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=96.70,588.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=42.58,672.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=127.88,451.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=183.65,489.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=251.92,471.87");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=245.03,539.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=317.75,539.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=387.41,472.33");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=323.08,446.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=423.56,407.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=366.90,360.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=526.97,417.29");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=498.89,496.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=535.55,581.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=469.21,641.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=539.95,700.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=410.52,621.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=391.10,724.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=361.98,669.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=212.26,635.61");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=164.76,759.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=86.96,766.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=6.67,762.46");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=63.19,857.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=160.47,822.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=214.65,866.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=235.42,776.88");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=326.14,752.99");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=364.72,834.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=436.53,895.20");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=462.06,990.58");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=514.54,959.69");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=592.67,928.28");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=574.55,995.37");
    }

}

